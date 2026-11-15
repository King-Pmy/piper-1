/**
  * @file    dm_motor.h
  * @brief   达妙 DM-J4310-2EC 电机驱动.
  * @version 1.1.0
  * @date    2026-10-31
  * @copyright Copyright (c) 2026 NERV Contributors
  *
  * SPDX-License-Identifier: MIT
  */

#ifndef DM_MOTOR_H_
#define DM_MOTOR_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 达妙控制帧与反馈帧的数据长度, 单位 byte. */
#define DM_MOTOR_FRAME_LEN 8U

/** @brief 电机 CAN ID 最小值. */
#define DM_MOTOR_ID_MIN 1U

/** @brief 电机 CAN ID 最大值. */
#define DM_MOTOR_ID_MAX 0x7FU

/** @brief 反馈帧 Master ID 最大值. */
#define DM_MOTOR_MASTER_ID_MAX 0x7FU

/** @brief 清除故障命令码. */
#define DM_MOTOR_OPCODE_CLEAR_ERROR 0xFBU

/** @brief 使能电机命令码. */
#define DM_MOTOR_OPCODE_ENABLE 0xFCU

/** @brief 失能电机命令码. */
#define DM_MOTOR_OPCODE_DISABLE 0xFDU

/** @brief 保存零点命令码. */
#define DM_MOTOR_OPCODE_SAVE_ZERO 0xFEU

/** @brief DM-J4310-2EC 位置满量程, 单位 rad. */
#define DM_MOTOR_P_MAX_RAD 12.5f

/** @brief DM-J4310-2EC 速度满量程, 单位 rad/s. */
#define DM_MOTOR_V_MAX_RADPS 30.0f

/** @brief DM-J4310-2EC 转矩满量程, 单位 N·m. */
#define DM_MOTOR_T_MAX_N_M 10.0f

/** @brief DM-J4310-2EC 位置刚度满量程. */
#define DM_MOTOR_KP_MAX 500.0f

/** @brief DM-J4310-2EC 速度阻尼满量程. */
#define DM_MOTOR_KD_MAX 5.0f

/** @brief 驱动器状态: 失能. 供应用层与 measurement.status_code 比较. */
#define DM_MOTOR_STATUS_DISABLED 0U

/** @brief 驱动器状态: 使能. 供应用层与 measurement.status_code 比较. */
#define DM_MOTOR_STATUS_ENABLED 1U

/** @brief 驱动器状态码: 超压. */
#define DM_MOTOR_STATUS_CODE_OVER_VOLTAGE 0x8U

/** @brief 驱动器状态码: 欠压. */
#define DM_MOTOR_STATUS_CODE_UNDER_VOLTAGE 0x9U

/** @brief 驱动器状态码: 过电流. */
#define DM_MOTOR_STATUS_CODE_OVER_CURRENT 0xAU

/** @brief 驱动器状态码: MOS 过温. */
#define DM_MOTOR_STATUS_CODE_MOS_OVER_TEMP 0xBU

/** @brief 驱动器状态码: 电机线圈过温. */
#define DM_MOTOR_STATUS_CODE_COIL_OVER_TEMP 0xCU

/** @brief 驱动器状态码: 通讯丢失. */
#define DM_MOTOR_STATUS_CODE_COMM_LOST 0xDU

/** @brief 驱动器状态码: 过载. */
#define DM_MOTOR_STATUS_CODE_OVERLOAD 0xEU

/** @brief 驱动器故障状态码下限. */
#define DM_MOTOR_STATUS_FAULT_MIN 8U

/** @brief 驱动器故障状态码上限. */
#define DM_MOTOR_STATUS_FAULT_MAX 0xEU

/** @brief 判断驱动器状态码是否表示故障. */
#define DM_MOTOR_STATUS_IS_FAULT(status) \
    (((status) >= DM_MOTOR_STATUS_FAULT_MIN) && ((status) <= DM_MOTOR_STATUS_FAULT_MAX))

/**
  * @brief 达妙控制模式.
  * @note  数值与 0x7FF 模式切换帧的 D[4] 一致.
  * @note  同时用作 dm_motor_cmd_t 的联合体标签, 与成员一一对应.
  */
typedef enum
{
    DM_MOTOR_MODE_MIT               = 1, /**< MIT 模式 */
    DM_MOTOR_MODE_POSITION_VELOCITY = 2, /**< 位置速度模式 */
    DM_MOTOR_MODE_VELOCITY          = 3, /**< 速度模式 */
} DM_MOTOR_MODE;

/**
  * @brief 驱动返回状态.
  */
typedef enum
{
    DM_OK    = 0, /**< 操作成功 */
    DM_ERROR = 1, /**< 通用错误 */
} dm_status_t;

/**
  * @brief 电机测量值, 机构系.
  * @note  达妙协议本身就是输出轴量, 电机内置的 10:1 减速比已包含在内.
  * @note  角度、角速度与转矩已按应用侧传动比换算, 并按安装方向取反.
  * @note  角度不做归一化, 原样保留驱动上报的输出轴角度, 范围约 ±P_MAX.
  * @note  转矩为理想输出转矩, 未计入传动效率.
  */
typedef struct
{
    float angle_rad;          /**< 输出轴角度, 单位 rad, 范围约 ±P_MAX, 未归一化 */
    float speed_radps;        /**< 输出轴角速度, 单位 rad/s */
    float torque_n_m;         /**< 输出轴转矩, 单位 N·m */
    int8_t temperature_c;     /**< 线圈温度, 单位 ℃ */
    int8_t temperature_mos_c; /**< MOS 温度, 单位 ℃ */
    uint8_t status_code;      /**< ERR 状态码, 反馈帧 D[0] 高 4 位, 0 失能, 1 使能, 8..E 故障 */
    uint8_t device_id;        /**< 电机 ID, 反馈帧 D[0] 低 4 位, 即 device_id 的低 4 位 */
} dm_motor_measurement_t;

/**
  * @brief 电机控制目标, 达妙输出轴坐标系.
  * @note  联合体只保留一份有效成员, 由 kind 标识.
  * @note  gear_ratio 只用于反馈换算, 不参与指令换算. 需要机构系目标时,
  *        调用方应按 gear_ratio 先换算成达妙输出轴量再传入.
  */
typedef struct
{
    DM_MOTOR_MODE kind; /**< 联合体标签, 取 DM_MOTOR_MODE_* */
    union
    {
        float velocity_radps; /**< 速度模式分支, 单位 rad/s */
        struct
        {
            float position_rad;   /**< 目标位置, 单位 rad */
            float velocity_radps; /**< 目标速度, 单位 rad/s */
            float kp;             /**< 位置刚度 */
            float kd;             /**< 速度阻尼 */
            float torque_n_m;     /**< 前馈转矩, 单位 N·m */
        } mit;
        struct
        {
            float position_rad;   /**< 目标位置, 单位 rad */
            float velocity_radps; /**< 限速, 单位 rad/s, 按绝对值使用 */
        } position_velocity;
    } u;
} dm_motor_cmd_t;

/**
  * @brief 电机实例配置.
  * @note  内容在 dm_motor_init 中复制, 调用方无需保持生命周期.
  * @note  一个实例只能由一个消费者持有.
  */
typedef struct
{
    uint8_t device_id;          /**< 电机 CAN ID, 范围 1 到 127 */
    uint8_t master_id;          /**< 反馈帧 ID, 范围 0 到 127, 同一总线必须唯一 */
    DM_MOTOR_MODE mode;         /**< 控制模式, 决定控制帧标识符与联合体标签 */
    float gear_ratio;           /**< 额外安装的减速箱的减速比, 无外部减速箱的情况下写 1.0f */
    uint32_t offline_ms;        /**< 离线判定阈值, 单位 ms, 0 表示关闭 */
    float velocity_limit_radps; /**< 速度指令限幅, 单位 rad/s, 0 表示不额外限幅 */
    bool invert_direction;      /**< 安装方向取反, 指令与反馈统一乘符号 */
} dm_motor_config_t;

/**
  * @brief 一帧标准 CAN 数据.
  * @note  本模块不直接访问 FDCAN 外设, 只写本结构, 由调用方发送.
  */
typedef struct
{
    uint32_t id;                      /**< 11 位标准帧标识符 */
    uint8_t len;                      /**< 数据长度, 单位 byte */
    uint8_t data[DM_MOTOR_FRAME_LEN]; /**< 数据区 */
} dm_frame_t;

/**
  * @brief 电机实例.
  * @note  由调用方提供存储, 生命周期覆盖全部调用; dm_motor_init 会清零实例.
  * @note  config 之后的字段由模块维护, 调用方只读.
  */
typedef struct
{
    dm_motor_config_t config;           /**< 配置, 由 dm_motor_init 绑定 */
    dm_motor_measurement_t measurement; /**< 机构系测量值, 由 dm_motor_on_rx 刷新 */
    uint32_t tx_id;                     /**< 控制帧标识符, 由 dm_motor_init 算出 */
    uint32_t last_rx_ms;                /**< 最近一次收到反馈帧的时刻, 单位 ms */
    bool offline_armed;                 /**< 离线计时起点已建立 */
    bool offline;                       /**< 是否离线, 超过 offline_ms 没收到反馈, 收到即恢复 */
    bool initialized;                   /**< 初始化完成标志 */
} dm_motor_t;

dm_status_t dm_motor_init(dm_motor_t *motor, const dm_motor_config_t *config);
dm_status_t dm_motor_set_velocity(dm_motor_t *motor, float velocity_radps, dm_frame_t *frame);
dm_status_t dm_motor_set_target(dm_motor_t *motor, const dm_motor_cmd_t *target, dm_frame_t *frame);
dm_status_t dm_motor_refresh_offline(dm_motor_t *motor, uint32_t now_ms);
dm_status_t dm_motor_change_mode(uint8_t device_id, DM_MOTOR_MODE mode, dm_frame_t *frame);
dm_status_t dm_motor_cmd_enable(const dm_motor_t *motor, dm_frame_t *frame);
dm_status_t dm_motor_cmd_disable(const dm_motor_t *motor, dm_frame_t *frame);
dm_status_t dm_motor_cmd_clear_error(const dm_motor_t *motor, dm_frame_t *frame);
dm_status_t dm_motor_cmd_save_zero(const dm_motor_t *motor, dm_frame_t *frame);
void dm_motor_on_rx(dm_motor_t *motor, uint32_t id, const uint8_t *data, uint8_t len,
                    uint32_t now_ms);
dm_status_t dm_motor_get_measurement(const dm_motor_t *motor, dm_motor_measurement_t *measurement);

#ifdef __cplusplus
}
#endif

#endif /* DM_MOTOR_H_ */
