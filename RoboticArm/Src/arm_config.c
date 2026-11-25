/**
  * @file    arm_config.c
  * @brief   机械臂应用配置的具体数值.
  *
  * 修改说明:
  *   - device_id / master_id 必须与电机实际拨码或达妙助手里设置的一致;
  *   - 同一总线上的 device_id 必须唯一, master_id 也必须唯一;
  *   - invert_direction 按实际安装方向填, 反装设 true;
  *   - g_q_zero 是"机械零位对应的电机角", 电机里已 save_zero 时保持 0.
  */

#include "arm_config.h"

const dm_motor_config_t arm_motor_configs[ARM_DOF] = {
    /* 关节1 - 底盘偏航 */
    {
        .device_id            = 1,                  /* 控制帧 ID 基值 + device_id */
        .master_id            = 0x11,               /* 反馈帧 ID, 与控制帧 ID 不重叠 */
        .mode                 = DM_MOTOR_MODE_MIT,  /* 控制模式 */
        .gear_ratio           = 1.0f,               /* 外部减速比 */
        .offline_ms           = 100,                /* 离线判定阈值 */
        .velocity_limit_radps = 15.0f,              /* 速度/PV 模式限速; MIT 不受此项限制 */
        .invert_direction     = false               /* 反装设 true */
    },
    /* 关节2 - 肩部俯仰 */
    { .device_id = 2, .master_id = 0x12, .mode = DM_MOTOR_MODE_MIT, .gear_ratio = 0.5f,
      .offline_ms = 100, .velocity_limit_radps = 15.0f, .invert_direction = false },
    /* 关节3 - 肘部俯仰 */
    { .device_id = 3, .master_id = 0x13, .mode = DM_MOTOR_MODE_MIT, .gear_ratio = 0.5f,
      .offline_ms = 100, .velocity_limit_radps = 15.0f, .invert_direction = false },
    /* 关节4 - 小臂旋转 */
    { .device_id = 4, .master_id = 0x14, .mode = DM_MOTOR_MODE_MIT, .gear_ratio = 1.0f,
      .offline_ms = 100, .velocity_limit_radps = 15.0f, .invert_direction = true },
    /* 关节5 - 腕部俯仰 */
    { .device_id = 5, .master_id = 0x15, .mode = DM_MOTOR_MODE_MIT, .gear_ratio = 1.0f,
      .offline_ms = 100, .velocity_limit_radps = 15.0f, .invert_direction = false },
    /* 关节6 - 腕部旋转 */
    { .device_id = 6, .master_id = 0x16, .mode = DM_MOTOR_MODE_MIT, .gear_ratio = 1.0f,
      .offline_ms = 100, .velocity_limit_radps = 15.0f, .invert_direction = false },
    /* 关节7 - 末端夹爪, 位置速度模式 */
    { .device_id = 7, .master_id = 0x17, .mode = DM_MOTOR_MODE_POSITION_VELOCITY, .gear_ratio = 1.0f,
      .offline_ms = 100, .velocity_limit_radps = 15.0f, .invert_direction = false }
};

const ArmJointGain_t g_arm_gain = {
    .kp = {30.0f, 30.0f, 20.0f, 15.0f, 10.0f, 10.0f, 0.0f},
    .kd = { 2.0f,  2.0f,  1.0f,  1.0f,  0.5f,  0.5f, 0.0f},
};

const ArmJointLimit_t g_arm_limit = {
    .min_rad = {-3.0f, -2.6f, -2.6f, -3.0f, -2.0f, -3.0f},
    .max_rad = { 3.0f,  2.6f,  2.6f,  3.0f,  2.0f,  3.0f},
};

const float g_q_zero[ARM_JOINT_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
