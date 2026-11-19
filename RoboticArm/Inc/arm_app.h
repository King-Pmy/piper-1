#ifndef ARM_APP_H
#define ARM_APP_H

#include "stm32h7xx_hal.h"
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

#endif /* ARM_APP_H */
