/**
 * @file    arm_gravity.h
 * @brief   六轴机械臂重力前馈（查表 + 双线性插值 + 摩擦 + 限幅 + 斜坡）
 *
 * 数据来源：由 gen_gravity_table.py 从 URDF 生成，编译进 arm_gravity_table.h。
 * 只覆盖 q2、q3 两个关节（本机 g1 恒为 0，g4/g6 近似为 0，g5 由夹爪引起、暂不补）。
 *
 * 用法（1 kHz 控制周期）：
 *   1) 上电 ArmGravity_Init + ArmGravity_Enable(1)；
 *   2) 每拍调 ArmGravity_Update(q_meas, qd_meas, dt, tau_ff)；
 *   3) 把 tau_ff[] 填进 MIT 帧的 torque_n_m。
 *
 * 注意：输入必须是实测关节角/角速度（不是指令），否则位置环滞后会形成正反馈。
 */

#ifndef ARM_GRAVITY_H
#define ARM_GRAVITY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ARM_GRAVITY_DOF 6

/** @brief 配置：表位置、量化比例、摩擦参数、限幅与斜坡。 */
typedef struct
{
    int      n;               /**< 每轴格点数 */
    float    q_min;           /**< 表覆盖下限，rad */
    float    q_max;           /**< 表覆盖上限，rad */
    float    scale;           /**< LSB -> N·m */
    const int16_t *tab2;      /**< g2 表，n*n，行=q2 */
    const int16_t *tab3;      /**< g3 表，n*n，行=q2 */
    float    fc[ARM_GRAVITY_DOF];      /**< 库仑摩擦，N·m */
    float    fv[ARM_GRAVITY_DOF];      /**< 粘滞摩擦，N·m·s/rad */
    float    tau_max[ARM_GRAVITY_DOF]; /**< 每关节输出限幅，N·m */
    float    slew_rate;       /**< 力矩斜率限制，N·m/s */
    float    qd_eps;          /**< 摩擦符号的死区，rad/s */
} ArmGravity_Cfg;

/** @brief 运行状态。 */
typedef struct
{
    ArmGravity_Cfg cfg;
    float    tau_ff[ARM_GRAVITY_DOF];  /**< 上一拍输出，用于斜坡 */
    uint8_t  enabled;
} ArmGravity_t;

/** @brief 用编译进来的表填一份默认配置（摩擦为 0，限幅 8 N·m，斜坡 50 N·m/s）。 */
void ArmGravity_DefaultCfg(ArmGravity_Cfg *cfg);

/** @brief 初始化实例；返回 0 成功，-1 参数非法。 */
int ArmGravity_Init(ArmGravity_t *g, const ArmGravity_Cfg *cfg);

/** @brief 使能/失能；失能时输出清零。 */
void ArmGravity_Enable(ArmGravity_t *g, int on);

/** @brief 清空输出与内部状态（上电、故障恢复时用）。 */
void ArmGravity_Reset(ArmGravity_t *g);

/**
 * @brief 纯重力项（不含摩擦、限幅），主要给测试与调试用。
 * @param grav 输出 6 路重力矩，N·m
 */
void ArmGravity_Gravity(const ArmGravity_t *g, const float q[ARM_GRAVITY_DOF],
                        float grav[ARM_GRAVITY_DOF]);

/**
 * @brief 每拍更新一次前馈力矩。
 * @param q   实测关节角，rad
 * @param qd  实测关节角速度，rad/s
 * @param dt  周期，s（<=0 时跳过斜坡）
 * @param tau_ff 输出 6 路前馈力矩，N·m
 * @return 0 正常，-1 参数非法
 */
int ArmGravity_Update(ArmGravity_t *g, const float q[ARM_GRAVITY_DOF],
                      const float qd[ARM_GRAVITY_DOF], float dt,
                      float tau_ff[ARM_GRAVITY_DOF]);

/** @brief 编译进固件的表的 CRC（与生成器输出比对，防止新旧表混用）。 */
uint32_t ArmGravity_TableCrc(void);

#ifdef __cplusplus
}
#endif

#endif /* ARM_GRAVITY_H */
