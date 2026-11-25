/**
  * @file    arm_motor.c
  * @brief   一条 FDCAN 总线上的多电机管理.
  */

#include "arm_motor.h"
#include "fdcan_bsp.h"

/* 通用发送: 与电机数量无关, 目标 ID 已经在 frame 里. */
static bool ArmMotor_Send(FDCAN_HandleTypeDef *hcan, const dm_frame_t *frame)
{
    return FDCAN_SendStd(hcan, frame->id, frame->data, frame->len) == HAL_OK;
}

/* 当前模式未知, 三种模式的失能帧都发一遍. */
static void ArmMotor_DisableAnyMode(ArmMotorBus_t *bus, const dm_motor_config_t *cfg)
{
    static const DM_MOTOR_MODE modes[3] = {
        DM_MOTOR_MODE_MIT, DM_MOTOR_MODE_POSITION_VELOCITY, DM_MOTOR_MODE_VELOCITY,
    };
    dm_motor_config_t mode_cfg = *cfg;
    dm_motor_t tmp;
    dm_frame_t frame;

    for (uint8_t i = 0U; i < 3U; i++)
    {
        mode_cfg.mode = modes[i];

        if ((dm_motor_init(&tmp, &mode_cfg) == DM_OK) &&
            (dm_motor_cmd_disable(&tmp, &frame) == DM_OK))
        {
            (void)ArmMotor_Send(bus->hfdcan, &frame);
        }
    }
}

/* 建实例 -> 失能 -> 切模式 -> 重建实例 -> 使能. */
static bool ArmMotor_InitOne(ArmMotorBus_t *bus, uint8_t index)
{
    dm_frame_t frame;
    ArmJoint_t *joint = &bus->joint[index];
    const dm_motor_config_t *cfg = &bus->cfg[index];

    joint->state = ARM_JOINT_DISABLED;

    if (dm_motor_init(&joint->drv, cfg) != DM_OK)
    {
        return false;
    }

    /* 切模式前先失能; 当前模式未知, 三路都发. */
    ArmMotor_DisableAnyMode(bus, cfg);

    if (dm_motor_change_mode(cfg->device_id, cfg->mode, &frame) != DM_OK)
    {
        return false;
    }

    if (!ArmMotor_Send(bus->hfdcan, &frame))
    {
        return false;
    }

    /* 等这一台确认模式; 低频操作, 初始化阶段阻塞可以接受. */
    HAL_Delay(50U);

    /* 新模式的控制帧 ID 不同, 必须重建实例. */
    if (dm_motor_init(&joint->drv, cfg) != DM_OK)
    {
        return false;
    }

    return ArmMotorBus_EnableOne(bus, index);
}

/* -------------------------------------------------------------------------
 * 初始化 校验-建实例-建master_id-index分发表-启动总线-逐台初始化-失败回滚
 * ---------------------------------------------------------------------- */

bool ArmMotorBus_Init(ArmMotorBus_t *bus, FDCAN_HandleTypeDef *hfdcan,
                      const dm_motor_config_t *cfgs, uint8_t count)
{
    if ((bus == NULL) || (hfdcan == NULL) || (cfgs == NULL) ||
        (count == 0U) || (count > ARM_MOTOR_MAX_COUNT))
    {
        return false;//API检查，防joint数越界
    }

    *bus       = (ArmMotorBus_t){0};//清零旧配置
    bus->hfdcan  = hfdcan;
    bus->cfg   = cfgs;
    bus->count = count;

    for (uint16_t id = 0U; id < ARM_MOTOR_RX_INDEX_SIZE; id++)
    {
        bus->rx_index[id] = -1;
    }

    /* 建分发表, 同时检查 device_id / master_id 是否重复. */
    for (uint8_t i = 0U; i < count; i++)
    {
        if ((cfgs[i].device_id < DM_MOTOR_ID_MIN) ||
            (cfgs[i].device_id > DM_MOTOR_ID_MAX) ||
            (cfgs[i].master_id >= ARM_MOTOR_RX_INDEX_SIZE))
        { 
            return false;
        }

        if (bus->rx_index[cfgs[i].master_id] >= 0)
        {
            return false; /* master_id 在总线上唯一 */
        }

        for (uint8_t j = 0U; j < i; j++)
        {
            if (cfgs[j].device_id == cfgs[i].device_id)
            {
                return false; /* device_id不与master_id重复 */
            }
        }

        bus->rx_index[cfgs[i].master_id] = (int8_t)i;
    }

    /* 总线级操作只做一次, 不能放进下面的单台循环. */
    if (FDCAN_AddFilter(hfdcan, 0U, FDCAN_FILTER_MASK, 0x000U, 0x000U,
                        FDCAN_FILTER_TO_RXFIFO0) != HAL_OK)
    {
        return false;
    }

    if (FDCAN_BringUp(hfdcan) != HAL_OK)
    {
        return false;
    }

    for (uint8_t i = 0U; i < count; i++)
    {
        if (!ArmMotor_InitOne(bus, i))
        {
            (void)ArmMotorBus_DisableAll(bus);
            return false;
        }
    }

    return true;
}

/* -------------------------------------------------------------------------
 * 接收与周期刷新
 * ---------------------------------------------------------------------- */

void ArmMotorBus_OnRx(ArmMotorBus_t *bus, uint32_t id,
                      const uint8_t *data, uint8_t len, uint32_t now_ms)
{
    if ((bus == NULL) || (data == NULL) || (bus->count == 0U) ||
        (id >= ARM_MOTOR_RX_INDEX_SIZE))
    {
        return;
    }

    int8_t index = bus->rx_index[id];

    if (index < 0)
    {
        return; /* 未知 ID, 直接丢弃 */
    }

    dm_motor_on_rx(&bus->joint[(uint8_t)index].drv, id, data, len, now_ms);
}

/*每个控制周期刷新离线状态，并推进关节状态机*/
void ArmMotorBus_Update(ArmMotorBus_t *bus, uint32_t now_ms)
{
    if ((bus == NULL) || (bus->count == 0U))
    {
        return;
    }

    for (uint8_t i = 0U; i < bus->count; i++)
    {
        ArmJoint_t *joint = &bus->joint[i];
        uint8_t status;

        (void)dm_motor_refresh_offline(&joint->drv, now_ms);

        if (joint->drv.offline)
        {
            joint->state = ARM_JOINT_FAULT;
            continue;
        }

        status = joint->drv.measurement.status_code;

        if (DM_MOTOR_STATUS_IS_FAULT(status))
        {
            joint->state = ARM_JOINT_FAULT;
        }
        else if (status == DM_MOTOR_STATUS_ENABLED)
        {
            if (joint->state != ARM_JOINT_FAULT)
            {
                joint->state = ARM_JOINT_READY;
            }
        }
        else if (status == DM_MOTOR_STATUS_DISABLED)
        {
            if (joint->state != ARM_JOINT_FAULT)
            {
                joint->state = ARM_JOINT_DISABLED;
            }
        }
        else
        {
            joint->state = ARM_JOINT_FAULT;
        }
    }
}

/* -------------------------------------------------------------------------
 * 目标下发
 * ---------------------------------------------------------------------- */

bool ArmMotorBus_SetTargetOne(ArmMotorBus_t *bus, uint8_t index,
                              const dm_motor_cmd_t *cmd)
{
    dm_frame_t frame;

    if ((bus == NULL) || (cmd == NULL) || (index >= bus->count))
    {
        return false;
    }

    if (dm_motor_set_target(&bus->joint[index].drv, cmd, &frame) != DM_OK)
    {
        return false;
    }

    return ArmMotor_Send(bus->hfdcan, &frame);
}

/* -------------------------------------------------------------------------
 * 使能 / 失能
 * ---------------------------------------------------------------------- */

bool ArmMotorBus_EnableOne(ArmMotorBus_t *bus, uint8_t index)
{
    dm_frame_t frame;

    if ((bus == NULL) || (index >= bus->count))
    {
        return false;
    }

    if (dm_motor_cmd_enable(&bus->joint[index].drv, &frame) != DM_OK)
    {
        return false;
    }

    if (!ArmMotor_Send(bus->hfdcan, &frame))
    {
        return false;
    }

    bus->joint[index].state = ARM_JOINT_ENABLING;
    return true;
}

bool ArmMotorBus_DisableOne(ArmMotorBus_t *bus, uint8_t index)
{
    dm_frame_t frame;

    if ((bus == NULL) || (index >= bus->count))
    {
        return false;
    }

    if (dm_motor_cmd_disable(&bus->joint[index].drv, &frame) != DM_OK)
    {
        return false;
    }

    if (!ArmMotor_Send(bus->hfdcan, &frame))
    {
        return false;
    }

    bus->joint[index].state = ARM_JOINT_DISABLED;
    return true;
}

bool ArmMotorBus_DisableAll(ArmMotorBus_t *bus)
{
    bool ok = true;

    if ((bus == NULL) || (bus->count == 0U))
    {
        return false;
    }

    for (uint8_t i = 0U; i < bus->count; i++)
    {
        if (!ArmMotorBus_DisableOne(bus, i))
        {
            ok = false;
        }
    }

    return ok;
}

bool ArmMotorBus_ClearFaultOne(ArmMotorBus_t *bus, uint8_t index)
{
    dm_frame_t frame;

    if ((bus == NULL) || (index >= bus->count))
    {
        return false;
    }

    if (dm_motor_cmd_clear_error(&bus->joint[index].drv, &frame) != DM_OK)
    {
        return false;
    }

    if (!ArmMotor_Send(bus->hfdcan, &frame))
    {
        return false;
    }

    bus->joint[index].state = ARM_JOINT_DISABLED;
    return true;
}

/* -------------------------------------------------------------------------
 * 读取与状态查询
 * ---------------------------------------------------------------------- */

bool ArmMotorBus_GetFeedback(const ArmMotorBus_t *bus, uint8_t index,
                             dm_motor_measurement_t *out)
{
    if ((bus == NULL) || (out == NULL) || (index >= bus->count))
    {
        return false;
    }

    return dm_motor_get_measurement(&bus->joint[index].drv, out) == DM_OK;
}

bool ArmMotorBus_IsReady(const ArmMotorBus_t *bus, uint8_t index)
{
    const ArmJoint_t *joint;

    if ((bus == NULL) || (index >= bus->count))
    {
        return false;
    }

    joint = &bus->joint[index];

    return (joint->state == ARM_JOINT_READY);
}

ArmJointState_t ArmMotorBus_GetState(const ArmMotorBus_t *bus, uint8_t index)
{
    if ((bus == NULL) || (index >= bus->count))
    {
        return ARM_JOINT_FAULT;
    }

    return bus->joint[index].state;
}

bool ArmMotorBus_HasFault(const ArmMotorBus_t *bus)
{
    if ((bus == NULL) || (bus->count == 0U))
    {
        return true;
    }

    for (uint8_t i = 0U; i < bus->count; i++)
    {
        if (bus->joint[i].state == ARM_JOINT_FAULT)
        {
            return true;
        }
    }

    return false;
}
