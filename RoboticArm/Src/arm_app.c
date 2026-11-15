/**
  * @file    arm_app.c
  * @brief   机械臂应用层: 电机初始化、目标下发、接收分发、逆解封装.
  *
  * 分层:
  *   arm_app -> arm_motor -> dm_motor -> fdcan_bsp -> HAL
  *
  * 注意:
  *   - 控制任务每个周期先调用 Arm_Motor_Update(), 再调用 Arm_IK_GetQ() /
  *     Arm_SettargetPos(); 否则关节状态停留在 ENABLING, IsReady 会一直为 false.
  *   - 接收回调里调用 ArmMotorBus_OnRx(), 由管理层按 master_id 分发.
  */

#include "arm_app.h"
#include "dm_motor.h"
#include "arm_motor.h"
#include "main.h"

#include <stdbool.h>
#include <stdint.h>

#include "arm_ik.h"
#include "arm_config.h"

/* 一条 FDCAN 总线上的 7 台电机 */
static ArmMotorBus_t g_arm_bus;

/* ========================================================================
 * 电机初始化 / 周期刷新 / 目标下发
 * ===================================================================== */

bool Arm_Motor_Init(FDCAN_HandleTypeDef *hfdcan)
{
    return ArmMotorBus_Init(&g_arm_bus, hfdcan, arm_motor_configs, ARM_DOF);
}

void Arm_Motor_Update(uint32_t now_ms)
{
    ArmMotorBus_Update(&g_arm_bus, now_ms);
}

/**
  * @brief 下发 7 个电机的目标.
  * @param pos 电机目标角(或夹爪目标), 调用方传入的应是 "关节角 + q_zero".
  * @retval false 有电机未 READY, 或有命令构建失败.
  * @note  要求 7 台全部 READY 才发, 任意一台未就绪则整组不下发.
  */
bool Arm_SettargetPos(const float pos[ARM_DOF])
{
    dm_motor_cmd_t cmds[ARM_DOF] = {0};

    for (int i = 0; i < ARM_DOF; i++) {
        cmds[i].kind = arm_motor_configs[i].mode;

        if (cmds[i].kind == DM_MOTOR_MODE_MIT) {
            cmds[i].u.mit.position_rad   = pos[i];
            cmds[i].u.mit.velocity_radps = 0.0f;
            cmds[i].u.mit.kp             = g_arm_gain.kp[i];
            cmds[i].u.mit.kd             = g_arm_gain.kd[i];
            cmds[i].u.mit.torque_n_m     = 0.0f;
        } else if (cmds[i].kind == DM_MOTOR_MODE_POSITION_VELOCITY) {
            cmds[i].u.position_velocity.position_rad   = pos[i];
            cmds[i].u.position_velocity.velocity_radps =
                arm_motor_configs[i].velocity_limit_radps;
        } else {
            return false;
        }
    }

    return ArmMotorBus_SetTargets(&g_arm_bus, cmds);
}

/* ========================================================================
 * 接收分发
 * ===================================================================== */

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[64]; /* 按 FD 最大长度开, 防止 HAL 拷贝越界 */

    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U) {
        return;
    }

    while (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK) {
        if ((rx_header.IdType != FDCAN_STANDARD_ID) || (rx_header.DataLength > 8U)) {
            continue;
        }

        ArmMotorBus_OnRx(&g_arm_bus, rx_header.Identifier, rx_data,
                         (uint8_t)rx_header.DataLength, HAL_GetTick());
    }
}

/* ========================================================================
 * 逆解封装: 位姿 -> 最优关节角
 *
 * 对外只有一个函数:
 *   Arm_IK_GetQ(&target, q_out);
 *
 * 内部自动:
 *   - 第一次调用时用 arm_params 初始化 g_ik
 *   - 从 g_arm_bus.joint[0..5] 取实测角作为 q_seed
 *   - Solve -> Select -> Unwrap
 *
 * 说明:
 *   - pose 位置单位 m, 角度单位 rad; 欧拉角约定 ZYX:
 *         R = Rz(yaw) * Ry(pitch) * Rx(roll)
 *   - q_out 是 DH 关节角, 不是电机角; 加 q_zero 后才是电机目标
 *   - 不可达 / 超限位 / 电机未就绪 都返回 false, 上层保持上一拍
 * ===================================================================== */

static ArmIK_Model g_ik;
static bool g_ik_ready = false;

/* 第一次调用时初始化逆解模型 */
static bool IkEnsureReady(void)
{
    if (g_ik_ready) {
        return true;
    }

    if (ArmIK_Init(&g_ik, arm_params) != ARM_IK_OK) {
        return false;
    }

    /* 默认工具点是法兰; 装了夹爪要把 TCP 相对法兰的平移填在这里 */
    ArmIK_SetTool(&g_ik, 0.0f, 0.0f, 0.0f);
    g_ik_ready = true;
    return true;
}

bool Arm_IK_GetQ(const ArmPos_t *target, float q_out[ARM_JOINT_COUNT])
{
    float q_seed[ARM_JOINT_COUNT];

    if ((target == NULL) || (q_out == NULL) || (!IkEnsureReady())) {
        return false;
    }

    /* 用实测关节角做 seed; 任一台未就绪直接判失败 */
    for (int i = 0; i < ARM_JOINT_COUNT; i++) {
        dm_motor_measurement_t m;

        if (!ArmMotorBus_IsReady(&g_arm_bus, i)) {
            return false;
        }
        if (!ArmMotorBus_GetFeedback(&g_arm_bus, i, &m)) {
            return false;
        }

        q_seed[i] = m.angle_rad - g_q_zero[i];
    }

    /* 真正的 Solve -> Select -> Unwrap 在 arm_ik.c 里 */
    return ArmIK_SolvePose(&g_ik, target, q_seed,
                           g_arm_limit.min_rad, g_arm_limit.max_rad, q_out);
}
