/**
 * @file    arm_ik.c
 * @brief   6R 球形腕机械臂解析逆运动学（C99 / float）
 *
 * 几何分解（与 ik-geo 的 “spherical joint” 族一致）：
 *   1) 腕心 w = p - R * tool            —— 4/5/6 三轴交点（本机即法兰原点）
 *   2) 位置子问题（关节 1+2+3）
 *        w_x = cosf(t1) * rho
 *        w_y = sinf(t1) * rho
 *        rho = a2*cosf(t2) + a3*cosf(t2+t3) + d4*sinf(t2+t3)
 *        w_z = d1 + a2*sinf(t2) + a3*sinf(t2+t3) - d4*cosf(t2+t3)
 *      => 平面 2R：Rw = hypotf(a3,d4)，phi = atan2f(d4,a3)，psi = t2+t3-phi
 *         rho     = a2*cosf(t2) + Rw*cosf(psi)
 *         w_z-d1  = a2*sinf(t2) + Rw*sinf(psi)
 *   3) 姿态子问题（关节 4+5+6）
 *        R36 = R03^T * R,  R36 = Rx(pi/2)Rz(t4)Rx(-pi/2)Rz(t5)Rx(pi/2)Rz(t6)
 *        => t5 = ±acosf(-R36[1][2])，t4/t6 由 atan2f 直接给出
 *
 * 说明：正解与 4x4 矩阵运算在 arm_kinematics.c / arm_math.c，
 *       本文件不再重复实现（ArmIK_Residual 直接调 Arm_ForwardKinematics）。
 *       全部闭式：加减乘除、sqrtf、atan2f、acosf，无迭代。
 */

#include "arm_ik.h"

#include <math.h>

#define ARM_IK_PI            3.14159265358979f
#define ARM_IK_EPS_ANGLE     1e-6f   /* 角度/长度判零阈值（float 够用即可） */
#define ARM_IK_EPS_DEDUP     1e-6f   /* 两组解判定重复的阈值 */

/* ------------------------------------------------------------------ */
/* 内部小工具                                                          */
/* ------------------------------------------------------------------ */

/* 归一到 (-π, π]，避免 acos/atan2 组合出来的角度越界 */
static float wrap_pi(float x)
{
    if (x > ARM_IK_PI || x < -ARM_IK_PI)
        x = atan2f(sinf(x), cosf(x));
    return x;
}

/* acos 的定义域限幅：|c|>1 时夹到 ±1，保证不返回 NaN */
static float clamp_unit(float x)
{
    if (x > 1.0f)  return 1.0f;
    if (x < -1.0f) return -1.0f;
    return x;
}

/* 角度比较（用于 DH 几何前提检查） */
static int near_angle(float a, float b, float tol)
{
    return fabsf(wrap_pi(a - b)) < tol;
}

/* q(电机角) -> theta(MDH 内部角) */
static float theta_of(const ArmIK_Model *m, int i, float q)
{
    return m->dh[i].offset + m->dh[i].dir * q;
}

/* theta -> q（dir = ±1 时 q = dir*(theta-offset)） */
static float q_of_theta(const ArmIK_Model *m, int i, float theta)
{
    return m->dh[i].dir * (theta - m->dh[i].offset);
}

/* ------------------------------------------------------------------ */
/* 初始化                                                              */
/* ------------------------------------------------------------------ */

ArmIK_Status ArmIK_Init(ArmIK_Model *m, const MHD_Param_t dh[ARM_IK_DOF])
{
    int i;
    const float half_pi = ARM_IK_PI * 0.5f;

    if (!m || !dh)
        return ARM_IK_BAD_MODEL;

    /* 直接复制 arm_params，后面只读 */
    for (i = 0; i < ARM_IK_DOF; ++i)
        m->dh[i] = dh[i];

    /* 几何前提：alpha = {0, +π/2, 0, +π/2, -π/2, +π/2} */
    if (!near_angle(m->dh[0].alpha,  0.0f,    1e-4f) ||
        !near_angle(m->dh[1].alpha,  half_pi, 1e-4f) ||
        !near_angle(m->dh[2].alpha,  0.0f,    1e-4f) ||
        !near_angle(m->dh[3].alpha,  half_pi, 1e-4f) ||
        !near_angle(m->dh[4].alpha, -half_pi, 1e-4f) ||
        !near_angle(m->dh[5].alpha,  half_pi, 1e-4f)) {
        m->valid = 0;
        return ARM_IK_BAD_MODEL;
    }

    /* 4/5/6 必须交于一点：a4=a5=0、d5=0；a0=a1=0 保证 t1 只绕 z 转 */
    if (fabsf(m->dh[0].a) > 1e-4f || fabsf(m->dh[1].a) > 1e-4f ||
        fabsf(m->dh[4].a) > 1e-4f || fabsf(m->dh[5].a) > 1e-4f ||
        fabsf(m->dh[1].d) > 1e-4f || fabsf(m->dh[2].d) > 1e-4f ||
        fabsf(m->dh[4].d) > 1e-4f) {
        m->valid = 0;
        return ARM_IK_BAD_MODEL;
    }

    /* dir 必须是 ±1，否则 q 与 theta 不是一一对应 */
    for (i = 0; i < ARM_IK_DOF; ++i) {
        if (fabsf(fabsf(m->dh[i].dir) - 1.0f) > 1e-4f) {
            m->valid = 0;
            return ARM_IK_BAD_MODEL;
        }
    }

    /* 缓存逆解要用的四个几何量：
       d1 = 基座高度，a2 = 大臂长(带符号)，a3 = 小臂偏置，d4 = 腕心偏置 */
    m->d1 = m->dh[0].d;
    m->a2 = m->dh[2].a;
    m->a3 = m->dh[3].a;
    m->d4 = m->dh[3].d;
    m->Rw = hypotf(m->a3, m->d4);       /* 等效小臂长 */
    m->phi = atan2f(m->d4, m->a3);      /* 小臂相位角 */

    if (fabsf(m->a2) < 1e-6f || m->Rw < 1e-6f) {   /* 连杆长为 0 无解 */
        m->valid = 0;
        return ARM_IK_BAD_MODEL;
    }

    /* 默认工具点 = 沿 z6 的 d6；腕心与法兰的转换 */
    m->tool[0] = 0.0f;
    m->tool[1] = 0.0f;
    m->tool[2] = m->dh[5].d;

    m->valid = 1;
    return ARM_IK_OK;
}

void ArmIK_SetTool(ArmIK_Model *m, float x, float y, float z)
{
    if (!m) return;
    m->tool[0] = x;
    m->tool[1] = y;
    m->tool[2] = z;
}

/* ------------------------------------------------------------------ */
/* 逆解                                                                */
/* ------------------------------------------------------------------ */

/* 收解：把 theta 换算成 q、归一化、去重、入库
   out[]=解数组  *n=已有解数  thetas[]=6 个内部角  branch=肩肘腕组合  status=解质量 */
static void push_solution(const ArmIK_Model *m, ArmIK_Solution out[ARM_IK_MAX_SOLUTIONS],
                          int *n, const float thetas[6], unsigned char branch,
                          unsigned char status)
{
    int i, j;
    float q[ARM_IK_DOF];

    for (i = 0; i < ARM_IK_DOF; ++i)
        q[i] = wrap_pi(q_of_theta(m, i, thetas[i]));   /* 内部角 -> 关节角，归一到 (-π,π] */

    /* 去重：奇异时多个分支会重合，只留一份 */
    for (j = 0; j < *n; ++j) {
        int same = 1;
        for (i = 0; i < ARM_IK_DOF; ++i) {
            if (fabsf(wrap_pi(out[j].q[i] - q[i])) > ARM_IK_EPS_DEDUP) {
                same = 0;
                break;
            }
        }
        if (same)
            return;
    }
    if (*n >= ARM_IK_MAX_SOLUTIONS)                     /* 数组满则丢弃 */
        return;

    for (i = 0; i < ARM_IK_DOF; ++i)
        out[*n].q[i] = q[i];
    out[*n].branch = branch;
    out[*n].status = status;
    ++(*n);
}

int ArmIK_Solve(const ArmIK_Model *m, const float T[4][4],
                ArmIK_Solution out[ARM_IK_MAX_SOLUTIONS],
                const float q_seed[ARM_IK_DOF])
{
    float R[3][3], p[3], w[3];
    float A, Rw, phi, d1;
    float seed_t4 = 0.0f;
    int i, j, n = 0;

    if (!m || !m->valid || !T || !out)
        return 0;

    /* 拆出旋转阵和平移，T 与 Matrix4x4.data 同布局 */
    for (i = 0; i < 3; ++i) {
        for (j = 0; j < 3; ++j)
            R[i][j] = T[i][j];
        p[i] = T[i][3];
    }

    /* 腕心：目标点退回工具偏移（本机 d6=0，w 就是法兰原点） */
    for (i = 0; i < 3; ++i)
        w[i] = p[i] - (R[i][0] * m->tool[0] + R[i][1] * m->tool[1] + R[i][2] * m->tool[2]);

    A   = m->a2;
    Rw  = m->Rw;
    phi = m->phi;
    d1  = m->d1;

    if (q_seed)
        seed_t4 = m->dh[3].offset + m->dh[3].dir * q_seed[3];  /* 腕奇异时沿用上一拍 q4 */

    /* 肩分支：sh=0 正伸，sh=1 反伸（q1 转 180°，rho 取负） */
    for (int sh = 0; sh < 2; ++sh) {
        int sgn = (sh == 0) ? +1 : -1;
        float t1, rho, X, Y, Rn, k, c, base;
        unsigned char base_status = ARM_IK_OK;
        float r = hypotf(w[0], w[1]);      /* 腕心到 z 轴的水平距离 */

        if (r < ARM_IK_EPS_ANGLE) {
            /* 肩奇异：腕心在 1 轴线上，q1 任意 —— 只保留一支，q1 沿用 seed */
            if (sh == 1)
                continue;
            t1 = q_seed ? theta_of(m, 0, q_seed[0]) : m->dh[0].offset;
            rho = 0.0f;
            base_status = ARM_IK_SHOULDER_SINGULAR;
        } else {
            t1 = atan2f(w[1], w[0]) + (sgn > 0 ? 0.0f : ARM_IK_PI);
            rho = sgn * r;
        }

        /* 平面 2R：X 为肩到腕心的水平距离，Y 为相对基座高度 */
        X = rho;
        Y = w[2] - d1;
        Rn = hypotf(X, Y);
        if (Rn < ARM_IK_EPS_ANGLE)
            continue;

        /* X·cos(t2) + Y·sin(t2) = (X²+Y²+A²-Rw²)/(2A) = k */
        k = (X * X + Y * Y + A * A - Rw * Rw) / (2.0f * A);
        c = k / Rn;
        if (c > 1.0f || c < -1.0f)         /* 够不着：夹到边界，标记不可达 */
            base_status = ARM_IK_UNREACHABLE;
        c = clamp_unit(c);
        base = atan2f(Y, X);

        /* 肘分支：el=0 上解(+acos)，el=1 下解(-acos) */
        for (int el = 0; el < 2; ++el) {
            float t2 = base + (el == 0 ? 1.0f : -1.0f) * acosf(c);
            float psi = atan2f(Y - A * sinf(t2), X - A * cosf(t2));
            float t3 = psi + phi - t2;
            float R36[3][3];
            float s1, c1, s23, c23, c5, t5, s5;
            unsigned char st = base_status;

            /* R03 = A1A2A3 的旋转部分（解析式，省掉 3 次 4x4 乘法）：
               R03 = [ c23*c1, -c1*s23,  s1 ]
                     [ s1*c23, -s1*s23, -c1 ]
                     [ s23,     c23,     0  ]                        */
            s1  = sinf(t1);
            c1  = cosf(t1);
            s23 = sinf(t2 + t3);
            c23 = cosf(t2 + t3);

            /* R36 = R03^T * R */
            for (i = 0; i < 3; ++i) {
                float r03_0i = (i == 0) ? c23 * c1 : ((i == 1) ? (-c1 * s23) : s1);
                float r03_1i = (i == 0) ? s1 * c23  : ((i == 1) ? (-s1 * s23) : -c1);
                float r03_2i = (i == 0) ? s23       : ((i == 1) ? c23        : 0.0f);
                for (j = 0; j < 3; ++j)
                    R36[i][j] = r03_0i * R[0][j] + r03_1i * R[1][j] + r03_2i * R[2][j];
            }

            c5 = clamp_unit(-R36[1][2]);    /* cos(t5) = -R36[1][2] */
            if (fabsf(-R36[1][2]) > 1.0f)
                st = ARM_IK_UNREACHABLE;

            /* 腕分支：wr=0 / wr=1 即 q5 的 ± 两解（腕翻转） */
            for (int wr = 0; wr < 2; ++wr) {
                float t4, t6;
                unsigned char st2 = st;
                t5 = (wr == 0 ? 1.0f : -1.0f) * acosf(c5);
                s5 = sinf(t5);

                if (fabsf(s5) < 1e-5f) {
                    /* 腕奇异：t4/t6 只剩和或差，按上一拍 t4 分配 */
                    st2 = ARM_IK_WRIST_SINGULAR;
                    t4 = q_seed ? seed_t4 : m->dh[3].offset;
                    if (c5 > 0.0f) {
                        float sum = atan2f(R36[2][0], R36[0][0]);   /* t4 + t6 (q5=0) */
                        t6 = sum - t4;
                    } else {
                        float dif = atan2f(-R36[2][0], -R36[0][0]); /* t4 - t6 (q5=π) */
                        t6 = t4 - dif;
                    }
                } else {
                    t4 = atan2f(R36[2][2] / s5, R36[0][2] / s5);
                    t6 = atan2f(-R36[1][1] / s5, R36[1][0] / s5);
                }

                {
                    float thetas[6];
                    unsigned char branch = (unsigned char)((sh & 1) | ((el & 1) << 1) | ((wr & 1) << 2));
                    thetas[0] = t1;
                    thetas[1] = t2;
                    thetas[2] = t3;
                    thetas[3] = t4;
                    thetas[4] = t5;
                    thetas[5] = t6;
                    push_solution(m, out, &n, thetas, branch, st2);
                }
            }
        }
    }
    return n;
}

/* ------------------------------------------------------------------ */
/* 校验 / 选解                                                         */
/* ------------------------------------------------------------------ */

float ArmIK_Residual(const float T[4][4], const float q[ARM_IK_DOF])
{
    Matrix4x4 F;
    float e = 0.0f;
    float qq[ARM_IK_DOF];
    int i, j;

    if (!T || !q)
        return 1e9f;

    for (i = 0; i < ARM_IK_DOF; ++i)
        qq[i] = q[i];

    Arm_ForwardKinematics(qq, 0, &F);   /* 用你自己的正解回代, 只要矩阵 */

    for (i = 0; i < 4; ++i) {
        for (j = 0; j < 4; ++j) {
            float d = fabsf(F.data[i][j] - T[i][j]);
            if (d > e) e = d;
        }
    }
    return e;     /* < 1e-4 基本就是对的；偏大先查零点/方向/DH */
}

void ArmIK_Unwrap(float q[ARM_IK_DOF], const float q_pref[ARM_IK_DOF])
{
    int i;
    if (!q || !q_pref) return;
    for (i = 0; i < ARM_IK_DOF; ++i)          /* 取离上一拍最近的那一圈 */
        q[i] = q_pref[i] + wrap_pi(q[i] - q_pref[i]);
}

int ArmIK_Select(const ArmIK_Solution sol[], int n,
                 const float q_pref[ARM_IK_DOF],
                 const float q_min[ARM_IK_DOF],
                 const float q_max[ARM_IK_DOF],
                 const float w[ARM_IK_DOF])
{
    int i, k, best = -1;
    float best_cost = 0.0f;

    for (k = 0; k < n; ++k) {
        float cost = 0.0f;
        int ok = 1;

        for (i = 0; i < ARM_IK_DOF; ++i) {
            float q = sol[k].q[i];
            float wi = w ? w[i] : 1.0f;

            if (q_pref)                                   /* 先按上一拍展开 */
                q = q_pref[i] + wrap_pi(q - q_pref[i]);
            if (q_min && q < q_min[i]) { ok = 0; break; } /* 超限直接淘汰 */
            if (q_max && q > q_max[i]) { ok = 0; break; }
            if (q_pref)
                cost += wi * fabsf(q - q_pref[i]);        /* 关节空间行程 */
        }
        if (!ok)
            continue;
        if (sol[k].status == ARM_IK_UNREACHABLE)
            cost += 1e6f;                                 /* 优先精确解 */
        if (best < 0 || cost < best_cost) {
            best = k;
            best_cost = cost;
        }
    }
    return best;
}

/* 位姿 -> 最优关节角：Solve -> Select -> Unwrap 的一步封装。
 * 不依赖任何电机实例，seed/limits 由调用方传入。 */
bool ArmIK_SolvePose(const ArmIK_Model *model,
                     const ArmPos_t *target,
                     const float q_seed[ARM_IK_DOF],
                     const float q_min[ARM_IK_DOF],
                     const float q_max[ARM_IK_DOF],
                     float q_out[ARM_IK_DOF])
{
    Matrix4x4 T;
    ArmIK_Solution sol[ARM_IK_MAX_SOLUTIONS];
    int n;
    int best;

    if (!model || !target || !q_seed || !q_out)
        return false;

    Arm_PosToMatrix(target, &T);

    n = ArmIK_Solve(model, T.data, sol, q_seed);
    if (n <= 0)
        return false;

    best = ArmIK_Select(sol, n, q_seed, q_min, q_max, 0);
    if (best < 0)
        return false;

    if (sol[best].status == ARM_IK_UNREACHABLE)
        return false;

    ArmIK_Unwrap(sol[best].q, q_seed);

    for (int i = 0; i < ARM_IK_DOF; ++i)
        q_out[i] = sol[best].q[i];

    return true;
}
