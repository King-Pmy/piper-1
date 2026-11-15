/**
  * @file    arm_config.h
  * @brief   这台机械臂的应用配置: 电机 ID/模式/安装方向、MIT 增益、限位、零位.
  *
  * 约定:
  *   - 本文件只放类型定义和 extern 声明;
  *   - 具体数值写在 arm_config.c.
  *
  * 这样做的原因:
  *   如果在头文件里直接定义 const 变量, 任何包含它的 .c 都会生成一份副本,
  *   链接时会出现 multiple definition; 正确做法是头文件 extern, 源文件定义一次.
  */

#ifndef __ARM_CONFIG_H
#define __ARM_CONFIG_H

#include "dm_motor.h"

/* 7 个电机: 前 6 个是机械臂关节, 第 7 个是夹爪 */
#define ARM_DOF         7
#define ARM_JOINT_COUNT 6

/* 关节限位, 单位 rad, 按机构实际填写 */
typedef struct
{
    float min_rad[ARM_JOINT_COUNT];
    float max_rad[ARM_JOINT_COUNT];
} ArmJointLimit_t;

/* MIT 位置环增益; 前 6 轴用, 第 7 轴夹爪走位置速度模式, 填 0 即可 */
typedef struct
{
    float kp[ARM_DOF];
    float kd[ARM_DOF];
} ArmJointGain_t;

/** @brief 7 台电机的静态配置表, 下标与关节一一对应. */
extern const dm_motor_config_t arm_motor_configs[ARM_DOF];

/** @brief 前 6 轴 MIT 位置环增益. */
extern const ArmJointGain_t g_arm_gain;

/** @brief 6 轴关节限位, 单位 rad. */
extern const ArmJointLimit_t g_arm_limit;

/** @brief 机械零位对应的电机角, rad; 电机里已 save_zero 时保持 0. */
extern const float g_q_zero[ARM_JOINT_COUNT];

#endif /* ARM_CONFIG_H */
