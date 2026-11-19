#ifndef ARM_MOTOR_H
#define ARM_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include "dm_motor.h"

#include <stdbool.h>
#include <stdint.h>

#define ARM_MOTOR_MAX_COUNT 8U
#define ARM_MOTOR_RX_INDEX_SIZE 128U

typedef enum
{
    ARM_JOINT_DISABLED = 0,
    ARM_JOINT_ENABLING,
    ARM_JOINT_READY,
    ARM_JOINT_FAULT,
} ArmJointState_t;

typedef struct
{
    dm_motor_t      drv;
    ArmJointState_t state;
} ArmJoint_t;

typedef struct
{
    FDCAN_HandleTypeDef     *hcan;
    const dm_motor_config_t *cfg;
    ArmJoint_t               joint[ARM_MOTOR_MAX_COUNT];
    uint8_t                  count;
    int8_t                   rx_index[ARM_MOTOR_RX_INDEX_SIZE];
} ArmMotorBus_t;

/* 生命周期 */
bool ArmMotorBus_Init(ArmMotorBus_t *bus, FDCAN_HandleTypeDef *hcan,
                      const dm_motor_config_t *cfgs, uint8_t count);

/* 数据面 */
void ArmMotorBus_OnRx(ArmMotorBus_t *bus, uint32_t id,
                      const uint8_t *data, uint8_t len, uint32_t now_ms);
void ArmMotorBus_Update(ArmMotorBus_t *bus, uint32_t now_ms);
bool ArmMotorBus_GetFeedback(const ArmMotorBus_t *bus, uint8_t index,
                             dm_motor_measurement_t *out);

/* 控制面 */
bool ArmMotorBus_SetTargetOne(ArmMotorBus_t *bus, uint8_t index,
                              const dm_motor_cmd_t *cmd);
                              
bool ArmMotorBus_EnableOne(ArmMotorBus_t *bus, uint8_t index);
bool ArmMotorBus_DisableOne(ArmMotorBus_t *bus, uint8_t index);
bool ArmMotorBus_DisableAll(ArmMotorBus_t *bus);
bool ArmMotorBus_ClearFaultOne(ArmMotorBus_t *bus, uint8_t index);

/* 状态查询 */
bool ArmMotorBus_IsReady(const ArmMotorBus_t *bus, uint8_t index);
ArmJointSta  te_t ArmMotorBus_GetState(const ArmMotorBus_t *bus, uint8_t index);
bool ArmMotorBus_HasFault(const ArmMotorBus_t *bus);

#ifdef __cplusplus
}
#endif

#endif /* ARM_MOTOR_H */
