/**
  * @file    arm_app.c
  * @brief   机械臂应用层: 电机初始化、目标下发、接收分发.
  *
  * 控制任务每周期: Arm_Motor_Update() -> IsReady -> Arm_SettargetPos().
  */

#include "arm_app.h"
#include "dm_motor.h"
#include "arm_motor.h"
#include "main.h"

#include <stdbool.h>
#include <stdint.h>

#include "arm_config.h"

static ArmMotorBus_t g_arm_bus;

bool Arm_Motor_Init(FDCAN_HandleTypeDef *hfdcan)
{
    return ArmMotorBus_Init(&g_arm_bus, hfdcan, arm_motor_configs, ARM_DOF);
}

void Arm_Motor_Update(uint32_t now_ms)
{
    ArmMotorBus_Update(&g_arm_bus, now_ms);
}

bool Arm_SettargetPos(const float pos[ARM_DOF])
{
    dm_motor_cmd_t cmds[ARM_DOF] = {0};

    /* 6 轴全部就绪才动; 夹爪单独判断. */
    for (uint8_t i = 0U; i < ARM_JOINT_COUNT; i++)
    {
        if (!ArmMotorBus_IsReady(&g_arm_bus, i))
        {
            return false;
        }
    }

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

    for (uint8_t i = 0U; i < ARM_DOF; i++)
    {
        if ((i == ARM_JOINT_COUNT) &&
            (!ArmMotorBus_IsReady(&g_arm_bus, i)))
        {
            continue;
        }

        if (!ArmMotorBus_SetTargetOne(&g_arm_bus, i, &cmds[i]))
        {
            return false;
        }
    }

    return true;
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[64];

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
