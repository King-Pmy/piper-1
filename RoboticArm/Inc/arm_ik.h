/**
 * @file    arm_ik.h
 * @brief   6R 球形腕机械臂解析逆运动学（C99，float 版，配合 arm_kinematics.c 使用）
 *
 * 适用对象
 * --------
 * 6 轴全旋转机械臂，MDH 参数满足
 *      alpha = { 0, +pi/2, 0, +pi/2, -pi/2, +pi/2 }
 *      a0 = a1 = a4 = a5 = 0,  d2 = d3 = d5 = 0
 * 即：1 轴竖直、2/3 轴平行（肘）、4/5/6 轴交于一点（球形腕）。
 * 对应 ik-geo 分类中的 “Spherical joint” 族（Table I / Table II）。
 *
 * 与正解的分工
 * ------------
 * 本文件只管逆解；正解和 4x4 矩阵运算用 arm_math.c / arm_kinematics.c：
 *      Arm_ForwardKinematics()   —— 正解（也是 ArmIK_Residual 的回代工具）
 *      Matrix_Multiply4x4()      —— 矩阵乘（本文件已不再重复实现）
 *
 * 角度约定
 * --------
 *      theta_i = offset_i + dir_i * q_i        (dir_i = ±1)
 * 与你 arm_params[] 的定义一致，q 为电机侧关节角（rad）。
 *
 * 输出
 * ----
 * 一次最多 8 组解（肩 2 × 肘 2 × 腕 2），带奇异 / 不可达标记，无迭代、无 malloc。
 */

#ifndef __ARM_IK_H
#define __ARM_IK_H

#include "arm_kinematics.h"   /* MHD_Param_t、Matrix4x4、Arm_ForwardKinematics */
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ARM_IK_DOF            6
#define ARM_IK_MAX_SOLUTIONS  8

/* ------------------------------------------------------------------ */
/* 参数：逆解几何量（全部由 arm_params 缓存而来，单位 rad / m）          */
/* ------------------------------------------------------------------ */

/* 求解状态 / 解的质量标记 */
typedef enum {
    ARM_IK_OK               = 0,  /* 精确解 */
    ARM_IK_UNREACHABLE      = 1,  /* 超出工作空间：返回最近的最小二乘解 */
    ARM_IK_WRIST_SINGULAR   = 2,  /* 腕奇异 q5 = 0 或 π：q4/q6 只有和/差确定 */
    ARM_IK_SHOULDER_SINGULAR= 3,  /* 腕心落在 1 轴线上：q1 任意 */
    ARM_IK_BAD_MODEL        = 4   /* DH 不满足本闭式解的几何前提 */
} ArmIK_Status;

/* 一组逆解：q 为关节角(rad)，branch 标分支，status 标质量 */
typedef struct {
    float q[ARM_IK_DOF];        /* 关节角，已归一到 (-π, π] */
    unsigned char branch;       /* bit0 肩(0:+ / 1:-)，bit1 肘，bit2 腕 */
    unsigned char status;       /* ArmIK_Status */
} ArmIK_Solution;

/* 逆解模型：缓存四个几何量，避免每次求解重复算 */
typedef struct {
    MHD_Param_t dh[ARM_IK_DOF]; /* DH 表副本（含 offset / dir） */
    float d1;                   /* 基座高度 d1 */
    float a2;                   /* 大臂长 a2（带符号，本机为 -0.3583） */
    float a3;                   /* 小臂偏置 a3 */
    float d4;                   /* 腕心偏置 d4 */
    float Rw;                   /* 等效小臂长 = hypotf(a3, d4) */
    float phi;                  /* 小臂相位 = atan2f(d4, a3) */
    float tool[3];              /* 法兰原点 -> TCP（法兰系），默认 (0,0,d6) */
    int   valid;                /* 1 = DH 通过几何前提检查 */
} ArmIK_Model;

/* ------------------------------------------------------------------ */
/* 初始化 / 配置                                                       */
/* ------------------------------------------------------------------ */

/**
 * @brief 用 arm_params 初始化逆解模型（复制一份，不持有指针）。
 * @return ARM_IK_OK 或 ARM_IK_BAD_MODEL
 */
ArmIK_Status ArmIK_Init(ArmIK_Model *model, const MHD_Param_t dh[ARM_IK_DOF]);

/**
 * @brief 覆盖工具点偏移（法兰系），默认 (0,0,d6)。
 *        装了夹爪就填 TCP 相对法兰的平移：逆解先减掉它再对腕心求解。
 */
void ArmIK_SetTool(ArmIK_Model *model, float x, float y, float z);

/* ------------------------------------------------------------------ */
/* 逆解                                                                */
/* ------------------------------------------------------------------ */

/**
 * @brief 逆解：输入法兰位姿 T，输出全部解析解。
 * @param T       目标位姿，4x4 行主序（与 Matrix4x4.data 同布局）
 * @param out     至少 ARM_IK_MAX_SOLUTIONS 个元素
 * @param q_seed  可选，上一周期关节角；用于腕奇异时分配 q4、肩奇异时取 q1
 * @return 解的数量 (0..8)
 */
int ArmIK_Solve(const ArmIK_Model *model,
                const float T[4][4],
                ArmIK_Solution out[ARM_IK_MAX_SOLUTIONS],
                const float q_seed[ARM_IK_DOF]);

/* ------------------------------------------------------------------ */
/* 实用工具                                                            */
/* ------------------------------------------------------------------ */

/**
 * @brief 用正解回代检查某个解，返回 4x4 元素最大绝对误差。
 *        注意：内部调用 Arm_ForwardKinematics()，用的是全局 arm_params，
 *        因此 model 必须是 arm_params 初始化出来的那一份。
 */
float ArmIK_Residual(const float T[4][4], const float q[ARM_IK_DOF]);

/**
 * @brief 在候选解里挑“最省力”的一个：关节空间加权距离最小 + 满足限位。
 * @param q_pref / q_min / q_max / w 均可为 NULL（不限 / 等权）
 * @return 选中的下标，无合法解返回 -1
 */
int ArmIK_Select(const ArmIK_Solution sol[], int n,
                 const float q_pref[ARM_IK_DOF],
                 const float q_min[ARM_IK_DOF],
                 const float q_max[ARM_IK_DOF],
                 const float w[ARM_IK_DOF]);

/** 把解按 q_pref ± π 展开，保证关节连续（防 ±2π 跳变） */
void ArmIK_Unwrap(float q[ARM_IK_DOF], const float q_pref[ARM_IK_DOF]);

/**
 * @brief 位姿 -> 最优关节角（一步封装）。
 * @param target      目标位姿：位置 m，欧拉角 rad（ZYX: Rz*Ry*Rx）
 * @param q_seed      当前实测关节角，用于选解和奇异消歧
 * @param q_min/q_max 关节限位，可为 NULL
 * @param q_out       输出关节角（DH 关节角，不是电机角）
 * @return true 找到合法解；false 不可达 / 超限位 / 参数非法
 */
bool ArmIK_SolvePose(const ArmIK_Model *model,
                     const ArmPos_t *target,
                     const float q_seed[ARM_IK_DOF],
                     const float q_min[ARM_IK_DOF],
                     const float q_max[ARM_IK_DOF],
                     float q_out[ARM_IK_DOF]);

#ifdef __cplusplus
}
#endif

#endif /* ARM_IK_H */
