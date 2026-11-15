/**
  * @file    arm_motor.h
  * @brief   一条 FDCAN 总线上的多电机管理层.
  *
  * 分层:
  *   arm_app  ->  arm_motor(本模块)  ->  dm_motor  ->  fdcan_bsp  ->  HAL
  *
  * 本模块负责:
  *   - 持有 N 个 dm_motor_t 实例
  *   - master_id -> 实例 的接收分发
  *   - 总线级初始化(过滤器/Start) 与单台初始化(失能/切模式/使能)
  *   - 周期刷新离线状态与关节状态
  *   - 把 dm_frame_t 送到总线
  *
  * 本模块不负责:
  *   - 达妙协议编解码(在 dm_motor)
  *   - 运动学/轨迹/增益策略(在 arm_app)
  */

#ifndef ARM_MOTOR_H
#define ARM_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include "dm_motor.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/** @brief 一条总线最多挂多少台电机. */
#define ARM_MOTOR_MAX_COUNT 8U

/** @brief 反馈 ID 查表大小. 达妙 master_id 范围 0~0x7F. */
#define ARM_MOTOR_RX_INDEX_SIZE 128U

/**
  * @brief 单个关节的状态.
  * @note  FAULT 是锁存的, 需要显式 EnableOne 才能重新进入 ENABLING.
  */
typedef enum
{
    ARM_JOINT_DISABLED = 0, /**< 已失能/未使能 */
    ARM_JOINT_ENABLING,     /**< 已发使能帧, 等待反馈确认 */
    ARM_JOINT_READY,        /**< 反馈正常且驱动器已使能 */
    ARM_JOINT_FAULT,        /**< 离线、故障码或通信异常 */
} ArmJointState_t;

/**
  * @brief 一个关节: 驱动实例 + 关节层状态与零位.
  */
typedef struct
{
    dm_motor_t      drv;    /**< 达妙驱动实例 */
    ArmJointState_t state;  /**< 关节状态 */
    float           q_zero; /**< 机械零位对应的电机角, rad */
} ArmJoint_t;

/**
  * @brief 一条总线上的多电机集合.
  * @note  cfg 指向调用方的静态配置表, 本模块不复制.
  * @note  rx_index[master_id] = 实例下标, -1 表示没有实例.
  */
typedef struct
{
    FDCAN_HandleTypeDef     *hcan;   /**< 总线句柄 */
    const dm_motor_config_t *cfg;    /**< 配置表, count 项 */
    ArmJoint_t               joint[ARM_MOTOR_MAX_COUNT];
    uint8_t                  count;  /**< 实际电机数, 单台测试填 1 */
    int8_t                   rx_index[ARM_MOTOR_RX_INDEX_SIZE];
} ArmMotorBus_t;

/**
  * @brief 初始化一条总线上的全部电机.
  * @param bus   总线实例, 由调用方提供存储.
  * @param hcan  已由 CubeMX 初始化的 FDCAN 句柄.
  * @param cfgs  配置表, 生命周期需覆盖全部调用.
  * @param count 电机数量, 1 ~ ARM_MOTOR_MAX_COUNT.
  * @retval true  全部初始化并使能成功.
  * @retval false 参数非法, 总线启动失败, 或某台初始化失败(已尝试回滚失能).
  * @note  总线级操作(过滤器/Start)只做一次, 不要在循环里重复调用.
  * @note  NOT ISR-safe.
  */
bool ArmMotorBus_Init(ArmMotorBus_t *bus, FDCAN_HandleTypeDef *hcan,
                      const dm_motor_config_t *cfgs, uint8_t count);

/**
  * @brief 接收分发: 按反馈 ID 找到实例并解码.
  * @param bus    总线实例.
  * @param id     反馈帧标识符(master_id).
  * @param data   反馈帧数据.
  * @param len    数据长度, 单位 byte.
  * @param now_ms 当前时刻, 单位 ms.
  * @note  未知 ID 直接忽略.
  */
void ArmMotorBus_OnRx(ArmMotorBus_t *bus, uint32_t id,
                      const uint8_t *data, uint8_t len, uint32_t now_ms);

/**
  * @brief 周期刷新: 离线计时 + 关节状态机.
  * @param bus    总线实例.
  * @param now_ms 当前时刻, 单位 ms.
  * @note  建议与控制环同频调用, 例如 500 Hz 或 1 kHz.
  * @note  NOT ISR-safe.
  */
void ArmMotorBus_Update(ArmMotorBus_t *bus, uint32_t now_ms);

/**
  * @brief 下发单台目标.
  * @note  不做就绪检查, 供单轴调试/故障恢复使用; 正式控制请用 SetTargets.
  */
bool ArmMotorBus_SetTargetOne(ArmMotorBus_t *bus, uint8_t index,
                              const dm_motor_cmd_t *cmd);

/**
  * @brief 下发全部目标.
  * @param cmds 目标数组, 至少 count 项, 与实例一一对应.
  * @retval false 有电机未 READY, 或至少一帧发送失败.
  * @note  要求全部 READY 才发, 任意一台未就绪则整组不下发.
  */
bool ArmMotorBus_SetTargets(ArmMotorBus_t *bus,
                            const dm_motor_cmd_t cmds[]);

/** @brief 使能单台, 状态进入 ENABLING. */
bool ArmMotorBus_EnableOne(ArmMotorBus_t *bus, uint8_t index);

/** @brief 失能单台, 状态进入 DISABLED. */
bool ArmMotorBus_DisableOne(ArmMotorBus_t *bus, uint8_t index);

/** @brief 使能全部, 逐台调用 EnableOne. */
bool ArmMotorBus_EnableAll(ArmMotorBus_t *bus);

/** @brief 失能全部, 逐台调用 DisableOne. */
bool ArmMotorBus_DisableAll(ArmMotorBus_t *bus);

/**
  * @brief 读取单台最近一次测量值(电机系).
  * @retval false 参数非法或实例未初始化.
  */
bool ArmMotorBus_GetFeedback(const ArmMotorBus_t *bus, uint8_t index,
                             dm_motor_measurement_t *out);

/** @brief 单台是否 READY: 状态 READY + 在线 + 驱动器已使能. */
bool ArmMotorBus_IsReady(const ArmMotorBus_t *bus, uint8_t index);

/** @brief 是否全部 READY. */
bool ArmMotorBus_AllReady(const ArmMotorBus_t *bus);

/** @brief 是否存在 FAULT 关节. */
bool ArmMotorBus_HasFault(const ArmMotorBus_t *bus);

#ifdef __cplusplus
}
#endif

#endif /* ARM_MOTOR_H */
