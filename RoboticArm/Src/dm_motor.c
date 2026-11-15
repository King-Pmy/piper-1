/**
  * @file    dm_motor.c
  * @brief   达妙 DM-J4310-2EC 电机驱动实现.
  * @version 1.1.0
  * @date    2026-10-31
  * @copyright Copyright (c) 2026 NERV Contributors
  *
  * SPDX-License-Identifier: MIT
  */

#include "dm_motor.h"

#include <float.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

_Static_assert(sizeof(float) == 4U, "DM motor protocol requires a 32-bit float.");
_Static_assert((FLT_RADIX == 2) && (FLT_MANT_DIG == 24) && (FLT_MAX_EXP == 128) &&
                   (FLT_MIN_EXP == -125),
               "DM motor protocol requires IEEE-754 binary32 float.");

#define DM_MOTOR_POSITION_RAW_MAX   65535.0f
#define DM_MOTOR_AUX_RAW_MAX        4095.0f
#define DM_MOTOR_MODE_FRAME_ID      0x7FFU
#define DM_MOTOR_TX_ID_MIT          0x000U
#define DM_MOTOR_TX_ID_POSITION_VEL 0x100U
#define DM_MOTOR_TX_ID_VELOCITY     0x200U

/**
  * @brief 型号描述符.
  * @note  驱动内部类型, 只描述 DM-J4310-2EC 的协议量程.
  * @note  型号是电机本身的属性, 与控制模式无关; 各模式只是同一型号的不同控制帧格式.
  */
typedef struct
{
    float p_max_rad;   /**< 位置满量程, 单位 rad */
    float v_max_radps; /**< 速度满量程, 单位 rad/s */
    float t_max_n_m;   /**< 转矩满量程, 单位 N·m */
    float kp_max;      /**< 位置刚度满量程 */
    float kd_max;      /**< 速度阻尼满量程 */
} dm_model_t;

/** @brief DM-J4310-2EC 型号描述符. */
static const dm_model_t dm_model_j4310 = {
    .p_max_rad   = DM_MOTOR_P_MAX_RAD,
    .v_max_radps = DM_MOTOR_V_MAX_RADPS,
    .t_max_n_m   = DM_MOTOR_T_MAX_N_M,
    .kp_max      = DM_MOTOR_KP_MAX,
    .kd_max      = DM_MOTOR_KD_MAX,
};

/**
  * @brief  根据模式取控制帧标识符基值.
  * @param  mode 模式.
  * @param  base 输出基值, 不得为 NULL.
  * @retval true 模式合法, base 已写入.
  * @retval false 模式非法或 base 为 NULL.
  */
static bool dm_motor_tx_base(DM_MOTOR_MODE mode, uint32_t *base)
{
    if (NULL == base)
    {
        return false;
    }

    switch (mode)
    {
    case DM_MOTOR_MODE_MIT:
        *base = DM_MOTOR_TX_ID_MIT;
        return true;

    case DM_MOTOR_MODE_POSITION_VELOCITY:
        *base = DM_MOTOR_TX_ID_POSITION_VEL;
        return true;

    case DM_MOTOR_MODE_VELOCITY:
        *base = DM_MOTOR_TX_ID_VELOCITY;
        return true;

    default:
        return false;
    }
}

/**
  * @brief  判断模式是否受支持.
  * @param  mode 模式.
  * @return 受支持返回 true.
  * @note   受支持的模式都有对应的控制帧标识符基值.
  */
static bool dm_motor_mode_valid(DM_MOTOR_MODE mode)
{
    uint32_t base;

    return dm_motor_tx_base(mode, &base);
}

/**
  * @brief  把数值限制在给定区间内.
  * @param  value 待限制的数值.
  * @param  min_value 区间下界.
  * @param  max_value 区间上界.
  * @return 落在区间内的值.
  * @note   调用方必须保证 min_value 小于等于 max_value.
  */
static float dm_motor_clamp(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }

    if (value > max_value)
    {
        return max_value;
    }

    return value;
}

/**
  * @brief  给出安装方向对应的符号.
  * @param  config 实例配置.
  * @return invert_direction 为 true 时返回 -1.0f, 否则返回 1.0f.
  */
static float dm_motor_direction_sign(const dm_motor_config_t *config)
{
    return config->invert_direction ? -1.0f : 1.0f;
}

/**
  * @brief  由配置计算控制帧标识符.
  * @param  config 实例配置, 不得为 NULL.
  * @param  tx_id 输出控制帧标识符, 不得为 NULL.
  * @retval DM_OK 已写入.
  * @retval DM_ERROR 参数为 NULL, 模式非法或标识符越界.
  * @note   反馈帧标识符就是 config->master_id, 不另存.
  */
static dm_status_t dm_motor_compute_tx_id(const dm_motor_config_t *config, uint32_t *tx_id)
{
    uint32_t base;

    if ((NULL == config) || (NULL == tx_id))
    {
        return DM_ERROR;
    }

    if ((config->device_id < DM_MOTOR_ID_MIN) || (config->device_id > DM_MOTOR_ID_MAX) ||
        (config->master_id > DM_MOTOR_MASTER_ID_MAX))
    {
        return DM_ERROR;
    }

    if (!dm_motor_tx_base(config->mode, &base))
    {
        return DM_ERROR;
    }

    *tx_id = base + (uint32_t)config->device_id;

    return DM_OK;
}

/**
  * @brief  极性量映射到无符号定点, 范围 [-max, +max] 对应 [0, bit_max].
  * @param  value 物理量.
  * @param  max_value 满量程绝对值.
  * @param  bit_max 编码满量程.
  * @return 无符号编码.
  */
static uint32_t dm_motor_encode_bipolar(float value, float max_value, uint32_t bit_max)
{
    float normalized;

    if (!(max_value > 0.0f))
    {
        return 0U;
    }

    normalized = dm_motor_clamp(value / max_value, -1.0f, 1.0f);

    return (uint32_t)(((normalized * 0.5f) + 0.5f) * (float)bit_max + 0.5f);
}

/**
  * @brief  单极性量映射到无符号定点, 范围 [0, max] 对应 [0, bit_max].
  * @param  value 物理量.
  * @param  max_value 满量程.
  * @param  bit_max 编码满量程.
  * @return 无符号编码.
  */
static uint32_t dm_motor_encode_unipolar(float value, float max_value, uint32_t bit_max)
{
    float normalized;

    if (!(max_value > 0.0f))
    {
        return 0U;
    }

    normalized = dm_motor_clamp(value / max_value, 0.0f, 1.0f);

    return (uint32_t)(normalized * (float)bit_max + 0.5f);
}

/**
  * @brief  把浮点数按小端写入 4 字节.
  * @param  data 输出缓冲, 长度不小于 4.
  * @param  value 待写入的浮点数.
  * @note   使用 memcpy 取得位模式, 再按字节位移写入, 不依赖主机字节序.
  */
static void dm_motor_write_float_le(uint8_t *data, float value)
{
    uint32_t raw = 0U;

    memcpy(&raw, &value, sizeof(raw));

    data[0] = (uint8_t)(raw & 0xFFU);
    data[1] = (uint8_t)((raw >> 8U) & 0xFFU);
    data[2] = (uint8_t)((raw >> 16U) & 0xFFU);
    data[3] = (uint8_t)((raw >> 24U) & 0xFFU);
}

/**
  * @brief  把控制目标编码到达妙控制帧.
  * @param  cmd 控制目标, 不得为 NULL.
  * @param  data 输出数据区, 长度不小于 DM_MOTOR_FRAME_LEN, 不得为 NULL.
  * @retval DM_OK 编码成功.
  * @retval DM_ERROR 参数为 NULL 或联合体标签非法.
  * @note   MIT 模式按 p_des(16), v_des(12), Kp(12), Kd(12), t_ff(12) 位打包.
  *         速度模式按 v_des 四字节浮点小端写入.
  *         位置速度模式按 p_des 与 v_des 两个四字节浮点小端写入.
  */
static dm_status_t dm_motor_encode_tx(const dm_motor_cmd_t *cmd, uint8_t *data)
{
    const dm_model_t *model = &dm_model_j4310;
    uint32_t p_raw;
    uint32_t v_raw;
    uint32_t kp_raw;
    uint32_t kd_raw;
    uint32_t t_raw;

    if ((NULL == cmd) || (NULL == data))
    {
        return DM_ERROR;
    }

    switch (cmd->kind)
    {
    case DM_MOTOR_MODE_MIT:
        p_raw = dm_motor_encode_bipolar(cmd->u.mit.position_rad, model->p_max_rad,
                                        (uint32_t)DM_MOTOR_POSITION_RAW_MAX);
        v_raw = dm_motor_encode_bipolar(cmd->u.mit.velocity_radps, model->v_max_radps,
                                        (uint32_t)DM_MOTOR_AUX_RAW_MAX);
        kp_raw =
            dm_motor_encode_unipolar(cmd->u.mit.kp, model->kp_max, (uint32_t)DM_MOTOR_AUX_RAW_MAX);
        kd_raw =
            dm_motor_encode_unipolar(cmd->u.mit.kd, model->kd_max, (uint32_t)DM_MOTOR_AUX_RAW_MAX);
        t_raw = dm_motor_encode_bipolar(cmd->u.mit.torque_n_m, model->t_max_n_m,
                                        (uint32_t)DM_MOTOR_AUX_RAW_MAX);

        data[0] = (uint8_t)((p_raw >> 8U) & 0xFFU);
        data[1] = (uint8_t)(p_raw & 0xFFU);
        data[2] = (uint8_t)((v_raw >> 4U) & 0xFFU);
        data[3] = (uint8_t)(((v_raw & 0x0FU) << 4U) | ((kp_raw >> 8U) & 0x0FU));
        data[4] = (uint8_t)(kp_raw & 0xFFU);
        data[5] = (uint8_t)((kd_raw >> 4U) & 0xFFU);
        data[6] = (uint8_t)(((kd_raw & 0x0FU) << 4U) | ((t_raw >> 8U) & 0x0FU));
        data[7] = (uint8_t)(t_raw & 0xFFU);

        return DM_OK;

    case DM_MOTOR_MODE_VELOCITY:
        dm_motor_write_float_le(&data[0], cmd->u.velocity_radps);
        memset(&data[4], 0, 4U);

        return DM_OK;

    case DM_MOTOR_MODE_POSITION_VELOCITY:
        dm_motor_write_float_le(&data[0], cmd->u.position_velocity.position_rad);
        dm_motor_write_float_le(&data[4], cmd->u.position_velocity.velocity_radps);

        return DM_OK;

    default:
        return DM_ERROR;
    }
}

/**
  * @brief  把达妙反馈帧解码并换算为机构系测量值.
  * @param  motor 实例指针, 不得为 NULL.
  * @param  frame 反馈帧数据, 不得为 NULL.
  * @param  len 反馈帧长度, 必须等于 DM_MOTOR_FRAME_LEN.
  * @param  measurement 输出测量值, 不得为 NULL. 失败时不修改.
  * @retval DM_OK 解码换算成功.
  * @retval DM_ERROR 参数为 NULL 或长度不符.
  * @note   反馈帧布局: D[0] 高 4 位 ERR 状态码, 低 4 位电机 ID, D[1..2] 位置 16 位,
  *         D[3..4] 速度 12 位, D[4..5] 转矩 12 位, D[6] MOS 温度, D[7] 线圈温度.
  * @note   达妙反馈不含电流测量, 只上报转矩.
  * @note   角度、角速度除以 gear_ratio, 转矩乘以 gear_ratio, 再按安装方向取反.
  * @note   转矩为理想值, 未计入传动效率.
  */
static dm_status_t dm_motor_decode_measurement(const dm_motor_t *motor, const uint8_t *frame,
                                               uint8_t len, dm_motor_measurement_t *measurement)
{
    const dm_model_t *model = &dm_model_j4310;
    const dm_motor_config_t *config;
    uint32_t pos_raw;
    uint32_t vel_raw;
    uint32_t torque_raw;
    float angle_rad;
    float speed_radps;
    float torque_n_m;
    float sign;

    if ((NULL == motor) || (NULL == frame) || (NULL == measurement))
    {
        return DM_ERROR;
    }

    if (len != (uint8_t)DM_MOTOR_FRAME_LEN)
    {
        return DM_ERROR;
    }

    config = &motor->config;

    pos_raw    = ((uint32_t)frame[1] << 8U) | (uint32_t)frame[2];
    vel_raw    = ((uint32_t)frame[3] << 4U) | ((uint32_t)frame[4] >> 4U);
    torque_raw = (((uint32_t)frame[4] & 0x0FU) << 8U) | (uint32_t)frame[5];

    angle_rad =
        ((float)pos_raw / DM_MOTOR_POSITION_RAW_MAX) * (2.0f * model->p_max_rad) - model->p_max_rad;
    speed_radps = (((float)vel_raw / DM_MOTOR_AUX_RAW_MAX) * 2.0f - 1.0f) * model->v_max_radps;
    torque_n_m  = (((float)torque_raw / DM_MOTOR_AUX_RAW_MAX) * 2.0f - 1.0f) * model->t_max_n_m;

    sign = dm_motor_direction_sign(config);

    measurement->angle_rad         = (angle_rad / config->gear_ratio) * sign;
    measurement->speed_radps       = (speed_radps / config->gear_ratio) * sign;
    measurement->torque_n_m        = torque_n_m * config->gear_ratio * sign;
    measurement->temperature_c     = (int8_t)frame[7];
    measurement->temperature_mos_c = (int8_t)frame[6];
    measurement->status_code       = (uint8_t)(frame[0] >> 4U);
    measurement->device_id         = (uint8_t)(frame[0] & 0x0FU);

    return DM_OK;
}

/**
  * @brief  按安装方向与限幅要求生成实际发送的目标.
  * @param  config 实例配置, 不得为 NULL.
  * @param  target 用户目标, 不得为 NULL, kind 必须等于 config->mode.
  * @param  effective 输出目标, 不得为 NULL.
  * @retval DM_OK 已写入.
  * @retval DM_ERROR 参数非法或目标模式与配置不一致.
  * @note   方向取反只作用于物理量分支.
  * @note   速度模式按符号限幅; 位置速度模式的限速按绝对值使用, 不随方向取反.
  * @note   MIT 模式不受 velocity_limit_radps 限制.
  */
static dm_status_t dm_motor_effective_command(const dm_motor_config_t *config,
                                              const dm_motor_cmd_t *target,
                                              dm_motor_cmd_t *effective)
{
    float sign;
    float limit;

    if ((NULL == config) || (NULL == target) || (NULL == effective))
    {
        return DM_ERROR;
    }

    if (target->kind != config->mode)
    {
        return DM_ERROR;
    }

    *effective = *target;
    sign       = dm_motor_direction_sign(config);
    limit      = config->velocity_limit_radps;

    switch (target->kind)
    {
    case DM_MOTOR_MODE_MIT:
        effective->u.mit.position_rad *= sign;
        effective->u.mit.velocity_radps *= sign;
        effective->u.mit.torque_n_m *= sign;
        break;

    case DM_MOTOR_MODE_POSITION_VELOCITY:
        effective->u.position_velocity.position_rad *= sign;
        effective->u.position_velocity.velocity_radps =
            fabsf(effective->u.position_velocity.velocity_radps);

        if ((limit > 0.0f) && (effective->u.position_velocity.velocity_radps > limit))
        {
            effective->u.position_velocity.velocity_radps = limit;
        }
        break;

    case DM_MOTOR_MODE_VELOCITY:
        effective->u.velocity_radps *= sign;

        if (limit > 0.0f)
        {
            effective->u.velocity_radps =
                dm_motor_clamp(effective->u.velocity_radps, -limit, limit);
        }
        break;

    default:
        return DM_ERROR;
    }

    return DM_OK;
}

/**
  * @brief  按当前控制目标生成控制帧.
  * @param  motor 实例指针, 不得为 NULL.
  * @param  target 控制目标, 不得为 NULL.
  * @param  frame 输出帧, 不得为 NULL. 失败时不修改.
  * @retval DM_OK 控制帧已写入.
  * @retval DM_ERROR 参数非法, 实例未初始化, 目标模式不匹配或编码失败.
  * @note   达妙控制帧为标准帧, 8 字节.
  * @note   NOT ISR-safe.
  */
static dm_status_t dm_motor_build_command(const dm_motor_t *motor, const dm_motor_cmd_t *target,
                                          dm_frame_t *frame)
{
    dm_motor_cmd_t effective;
    uint8_t data[DM_MOTOR_FRAME_LEN] = {0};

    if ((NULL == motor) || (NULL == target) || (NULL == frame))
    {
        return DM_ERROR;
    }

    if (!motor->initialized)
    {
        return DM_ERROR;
    }

    if (DM_OK != dm_motor_effective_command(&motor->config, target, &effective))
    {
        return DM_ERROR;
    }

    if (DM_OK != dm_motor_encode_tx(&effective, data))
    {
        return DM_ERROR;
    }

    frame->id  = motor->tx_id;
    frame->len = (uint8_t)DM_MOTOR_FRAME_LEN;
    memcpy(frame->data, data, DM_MOTOR_FRAME_LEN);

    return DM_OK;
}

/**
  * @brief  用配置建立电机实例.
  * @param  motor 实例指针, 不得为 NULL.
  * @param  config 配置指针, 不得为 NULL. 内容被复制.
  * @retval DM_OK 配置已绑定.
  * @retval DM_ERROR 参数非法或配置越界.
  * @note   重复调用以新配置重建实例, 复位状态标志, 接收计时, 反馈与目标.
  * @note   NOT ISR-safe.
  */
dm_status_t dm_motor_init(dm_motor_t *motor, const dm_motor_config_t *config)
{
    uint32_t tx_id;

    if ((NULL == motor) || (NULL == config))
    {
        return DM_ERROR;
    }

    if ((!isfinite(config->gear_ratio)) || (config->gear_ratio <= 0.0f))
    {
        return DM_ERROR;
    }

    if ((!isfinite(config->velocity_limit_radps)) || (config->velocity_limit_radps < 0.0f))
    {
        return DM_ERROR;
    }

    if (DM_OK != dm_motor_compute_tx_id(config, &tx_id))
    {
        return DM_ERROR;
    }

    *motor = (dm_motor_t){
        .config      = *config,
        .tx_id       = tx_id,
        .initialized = true,
    };

    return DM_OK;
}

/**
  * @brief  设置目标速度并生成控制帧, 仅速度模式可用.
  * @param  motor 实例指针, 不得为 NULL.
  * @param  velocity_radps 目标速度, 单位 rad/s, 必须为有限值.
  * @param  frame 输出帧, 不得为 NULL. 失败时不修改.
  * @retval DM_OK 目标已保存, 控制帧已写入.
  * @retval DM_ERROR 参数非法, 实例未初始化或当前模式不是速度模式.
  * @note   实例限幅在构帧时进行, 保存的是未限幅的目标.
  * @note   NOT ISR-safe.
  */
dm_status_t dm_motor_set_velocity(dm_motor_t *motor, float velocity_radps, dm_frame_t *frame)
{
    dm_motor_cmd_t target;

    if ((NULL == motor) || (NULL == frame) || (!isfinite(velocity_radps)))
    {
        return DM_ERROR;
    }

    if (!motor->initialized)
    {
        return DM_ERROR;
    }

    if (DM_MOTOR_MODE_VELOCITY != motor->config.mode)
    {
        return DM_ERROR;
    }

    memset(&target, 0, sizeof(target));
    target.kind             = DM_MOTOR_MODE_VELOCITY;
    target.u.velocity_radps = velocity_radps;

    return dm_motor_set_target(motor, &target, frame);
}

/**
  * @brief  设置多字段控制目标并生成控制帧.
  * @param  motor 实例指针, 不得为 NULL.
  * @param  target 控制目标, 不得为 NULL.
  * @param  frame 输出帧, 不得为 NULL. 失败时不修改.
  * @retval DM_OK 目标已保存, 控制帧已写入.
  * @retval DM_ERROR 参数非法, 实例未初始化, 字段非法或模式不匹配.
  * @note   MIT 分支要求所有字段为有限值, 且 kp, kd 非负.
  * @note   只有构帧成功后才保存目标; 失败时实例中的目标保持不变.
  * @note   NOT ISR-safe.
  */
dm_status_t dm_motor_set_target(dm_motor_t *motor, const dm_motor_cmd_t *target, dm_frame_t *frame)
{
    if ((NULL == motor) || (NULL == target) || (NULL == frame))
    {
        return DM_ERROR;
    }

    if (!motor->initialized)
    {
        return DM_ERROR;
    }

    if (target->kind != motor->config.mode)
    {
        return DM_ERROR;
    }

    switch (target->kind)
    {
    case DM_MOTOR_MODE_VELOCITY:
        if (!isfinite(target->u.velocity_radps))
        {
            return DM_ERROR;
        }
        break;

    case DM_MOTOR_MODE_POSITION_VELOCITY:
        if ((!isfinite(target->u.position_velocity.position_rad)) ||
            (!isfinite(target->u.position_velocity.velocity_radps)))
        {
            return DM_ERROR;
        }
        break;

    case DM_MOTOR_MODE_MIT:
        if ((!isfinite(target->u.mit.position_rad)) || (!isfinite(target->u.mit.velocity_radps)) ||
            (!isfinite(target->u.mit.kp)) || (!isfinite(target->u.mit.kd)) ||
            (!isfinite(target->u.mit.torque_n_m)) || (target->u.mit.kp < 0.0f) ||
            (target->u.mit.kd < 0.0f))
        {
            return DM_ERROR;
        }
        break;

    default:
        return DM_ERROR;
    }

    return dm_motor_build_command(motor, target, frame);
}

/**
  * @brief  刷新离线状态.
  * @param  motor 实例指针, 不得为 NULL.
  * @param  now_ms 当前时刻, 单位 ms.
  * @retval DM_OK 状态已刷新.
  * @retval DM_ERROR 参数非法或实例未初始化.
  * @note   测量值由 dm_motor_on_rx 刷新, 本函数只判断是否超过 offline_ms 没收到反馈.
  * @note   离线状态写入 motor->offline, 不通过返回值报告; 收到新反馈时由 dm_motor_on_rx 清除.
  * @note   本函数不产生控制帧.
  * @note   NOT ISR-safe.
  */
dm_status_t dm_motor_refresh_offline(dm_motor_t *motor, uint32_t now_ms)
{
    if (NULL == motor)
    {
        return DM_ERROR;
    }

    if (!motor->initialized)
    {
        return DM_ERROR;
    }

    if ((!motor->offline_armed) || (now_ms < motor->last_rx_ms))
    {
        /* 首次判定或时间回退, 建立计时起点 */
        motor->last_rx_ms    = now_ms;
        motor->offline_armed = true;
        motor->offline       = false;

        return DM_OK;
    }

    if ((motor->config.offline_ms > 0U) &&
        ((now_ms - motor->last_rx_ms) >= motor->config.offline_ms))
    {
        motor->offline = true;
    }

    return DM_OK;
}

/**
  * @brief  生成达妙模式切换帧.
  * @param  device_id 电机 CAN ID, 范围 1 到 127.
  * @param  mode 模式, 取 DM_MOTOR_MODE_*.
  * @param  frame 输出帧, 不得为 NULL. 失败时不修改.
  * @retval DM_OK 模式帧已写入.
  * @retval DM_ERROR 参数非法或标识符越界.
  * @note   帧标识符固定 0x7FF, 数据为 device_id, 0x00, 0x55, 0x0A, mode, 0x00, 0x00, 0x00.
  * @note   本函数只生成帧, 实际切换在调用方发送后生效.
  * @note   切换流程建议: 先失能, 再写模式, 等待电机确认, 最后重新使能.
  * @note   NOT ISR-safe.
  */
dm_status_t dm_motor_change_mode(uint8_t device_id, DM_MOTOR_MODE mode, dm_frame_t *frame)
{
    if (NULL == frame)
    {
        return DM_ERROR;
    }

    if ((device_id < DM_MOTOR_ID_MIN) || (device_id > DM_MOTOR_ID_MAX))
    {
        return DM_ERROR;
    }

    if (!dm_motor_mode_valid(mode))
    {
        return DM_ERROR;
    }

    frame->id      = DM_MOTOR_MODE_FRAME_ID;
    frame->len     = (uint8_t)DM_MOTOR_FRAME_LEN;
    frame->data[0] = device_id;
    frame->data[1] = 0x00U;
    frame->data[2] = 0x55U;
    frame->data[3] = 0x0AU;
    frame->data[4] = (uint8_t)mode;
    frame->data[5] = 0x00U;
    frame->data[6] = 0x00U;
    frame->data[7] = 0x00U;

    return DM_OK;
}

/**
  * @brief  生成达妙特殊命令帧.
  * @param  motor 实例指针, 不得为 NULL.
  * @param  opcode 命令码, 取 DM_MOTOR_OPCODE_*.
  * @param  frame 输出帧, 不得为 NULL. 失败时不修改.
  * @retval DM_OK 命令帧已写入.
  * @retval DM_ERROR 参数非法或实例未初始化.
  * @note   命令帧的 CAN ID 等于当前控制模式的控制帧 ID, 数据为 7 字节 0xFF 加命令码.
  */
static dm_status_t dm_motor_build_opcode(const dm_motor_t *motor, uint8_t opcode, dm_frame_t *frame)
{
    if ((NULL == motor) || (NULL == frame))
    {
        return DM_ERROR;
    }

    if (!motor->initialized)
    {
        return DM_ERROR;
    }

    frame->id  = motor->tx_id;
    frame->len = (uint8_t)DM_MOTOR_FRAME_LEN;

    memset(frame->data, 0xFF, DM_MOTOR_FRAME_LEN - 1U);
    frame->data[DM_MOTOR_FRAME_LEN - 1U] = opcode;

    return DM_OK;
}

/**
  * @brief  生成使能帧.
  * @param  motor 实例指针, 不得为 NULL.
  * @param  frame 输出帧, 不得为 NULL. 失败时不修改.
  * @retval DM_OK 使能帧已写入.
  * @retval DM_ERROR 参数非法或实例未初始化.
  * @note   电机上电或掉线恢复后需要重新使能.
  * @note   NOT ISR-safe.
  */
dm_status_t dm_motor_cmd_enable(const dm_motor_t *motor, dm_frame_t *frame)
{
    return dm_motor_build_opcode(motor, DM_MOTOR_OPCODE_ENABLE, frame);
}

/**
  * @brief  生成失能帧.
  * @param  motor 实例指针, 不得为 NULL.
  * @param  frame 输出帧, 不得为 NULL. 失败时不修改.
  * @retval DM_OK 失能帧已写入.
  * @retval DM_ERROR 参数非法或实例未初始化.
  * @note   失能后电机不再输出力矩, 是切断输出的推荐方式.
  * @note   NOT ISR-safe.
  */
dm_status_t dm_motor_cmd_disable(const dm_motor_t *motor, dm_frame_t *frame)
{
    return dm_motor_build_opcode(motor, DM_MOTOR_OPCODE_DISABLE, frame);
}

/**
  * @brief  生成清除故障帧.
  * @param  motor 实例指针, 不得为 NULL.
  * @param  frame 输出帧, 不得为 NULL. 失败时不修改.
  * @retval DM_OK 清除故障帧已写入.
  * @retval DM_ERROR 参数非法或实例未初始化.
  * @note   清除驱动器锁存的故障码.
  * @note   NOT ISR-safe.
  */
dm_status_t dm_motor_cmd_clear_error(const dm_motor_t *motor, dm_frame_t *frame)
{
    return dm_motor_build_opcode(motor, DM_MOTOR_OPCODE_CLEAR_ERROR, frame);
}

/**
  * @brief  生成保存零点帧.
  * @param  motor 实例指针, 不得为 NULL.
  * @param  frame 输出帧, 不得为 NULL. 失败时不修改.
  * @retval DM_OK 保存零点帧已写入.
  * @retval DM_ERROR 参数非法或实例未初始化.
  * @note   把电机当前位置保存为机械零点, 发送时电机应保持静止.
  * @note   NOT ISR-safe.
  */
dm_status_t dm_motor_cmd_save_zero(const dm_motor_t *motor, dm_frame_t *frame)
{
    return dm_motor_build_opcode(motor, DM_MOTOR_OPCODE_SAVE_ZERO, frame);
}

/**
  * @brief  接收反馈帧并刷新机构系测量值.
  * @param  motor 实例指针, 不得为 NULL.
  * @param  id 反馈帧标识符.
  * @param  data 反馈帧数据, 不得为 NULL.
  * @param  len 反馈帧长度, 单位 byte.
  * @param  now_ms 当前时刻, 单位 ms, 用于刷新离线计时.
  * @note   标识符必须等于实例配置的 master_id, 长度必须为 8, 否则忽略.
  * @note   本函数直接解码并写入 measurement, 同时刷新 last_rx_ms 与 offline.
  * @note   驱动不做并发保护: 调用方必须保证本函数与 dm_motor_refresh_offline、
  *         dm_motor_get_measurement 不并发执行, 否则可能读到写了一半的测量值.
  * @note   本函数含浮点运算, 在中断里调用会占用一点时间.
  * @note   NOT ISR-safe.
  */
void dm_motor_on_rx(dm_motor_t *motor, uint32_t id, const uint8_t *data, uint8_t len,
                    uint32_t now_ms)
{
    if ((NULL == motor) || (NULL == data))
    {
        return;
    }

    if (!motor->initialized)
    {
        return;
    }

    if (id != motor->config.master_id)
    {
        return;
    }

    if (len != (uint8_t)DM_MOTOR_FRAME_LEN)
    {
        return;
    }

    if (DM_OK != dm_motor_decode_measurement(motor, data, len, &motor->measurement))
    {
        return;
    }

    motor->last_rx_ms    = now_ms;
    motor->offline_armed = true;
    motor->offline       = false;
}

/**
  * @brief  读取电机机构系测量值.
  * @param  motor 实例指针, 不得为 NULL.
  * @param  measurement 输出测量值, 不得为 NULL. 失败时不修改.
  * @retval DM_OK 测量值已写入.
  * @retval DM_ERROR 参数非法或实例未初始化.
  * @note   NOT ISR-safe.
  */
dm_status_t dm_motor_get_measurement(const dm_motor_t *motor, dm_motor_measurement_t *measurement)
{
    if ((NULL == motor) || (NULL == measurement))
    {
        return DM_ERROR;
    }

    if (!motor->initialized)
    {
        return DM_ERROR;
    }

    *measurement = motor->measurement;

    return DM_OK;
}
