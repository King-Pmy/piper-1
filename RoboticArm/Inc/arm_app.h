#ifndef ARM_APP_H
#define ARM_APP_H

#include "stm32h7xx_hal.h"
#include "arm_kinematics.h"
#include "dm_motor.h"
#include "arm_config.h"

#include <stdbool.h>
#include <stdint.h>

/* ARM_DOF / ARM_JOINT_COUNT / ArmJointLimit_t / ArmJointGain_t
 * 以及电机配置表、增益、限位、零位都在 arm_config.h */

/* 七电机: 初始化 / 周期刷新 / 目标 */
bool Arm_Motor_Init(FDCAN_HandleTypeDef *hfdcan);
void Arm_Motor_Update(uint32_t now_ms);
bool Arm_SettargetPos(const float pos[ARM_DOF]);

void Arm_Init(void);

void Arm_MoveToXYZ(float x, float y, float z, float duration);

void Arm_GetCurrentXYZ(float *x, float *y, float *z);

/* 逆解: 位姿 -> 最优关节角
 * 内部自动用 g_arm_bus.joint[0..5] 的实测角做 seed.
 * 调用前先 Arm_Motor_Update(), 否则关节状态可能还停在 ENABLING.
 * q_out 是关节角, 加 q_zero 后才是电机目标. */
bool Arm_IK_GetQ(const ArmPos_t *pos, float q_out[ARM_JOINT_COUNT]);

#endif /* ARM_APP_H */
