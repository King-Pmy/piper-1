/**
 * @file    arm_gravity.c
 * @brief   重力前馈实现：查表 + 双线性插值 + 摩擦 + 限幅 + 斜坡
 *
 * 表来自 arm_gravity_table.h（由 gen_gravity_table.py 生成）。
 */

#include "arm_gravity.h"
#include "arm_gravity_table.h"

#include <math.h>
#include <stddef.h>

static float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

/* 归一化到 [0, n-1-eps]，避免 i+1 越界 */
static float grid_pos(const ArmGravity_Cfg *c, float q)
{
    const float step = (c->q_max - c->q_min) / (float)(c->n - 1);
    float x = (q - c->q_min) / step;
    return clampf(x, 0.0f, (float)(c->n - 1) - 1e-4f);
}

/* 双线性插值，返回 N·m */
static float lookup(const ArmGravity_Cfg *c, const int16_t *tab, float q2, float q3)
{
    const float x = grid_pos(c, q2);
    const float y = grid_pos(c, q3);
    const int   i = (int)x;
    const int   j = (int)y;
    const float fx = x - (float)i;
    const float fy = y - (float)j;
    const int   n = c->n;

    const float t00 = (float)tab[i * n + j];
    const float t10 = (float)tab[(i + 1) * n + j];
    const float t01 = (float)tab[i * n + j + 1];
    const float t11 = (float)tab[(i + 1) * n + j + 1];

    const float t = (1.0f - fx) * (1.0f - fy) * t00
                  + fx * (1.0f - fy) * t10
                  + (1.0f - fx) * fy * t01
                  + fx * fy * t11;
    return t * c->scale;
}

/* ------------------------------------------------------------------ */
/* 公开接口                                                            */
/* ------------------------------------------------------------------ */

void ArmGravity_DefaultCfg(ArmGravity_Cfg *cfg)
{
    int i;

    if (cfg == NULL) {
        return;
    }

    cfg->n     = AG_N;
    cfg->q_min = AG_QMIN;
    cfg->q_max = AG_QMAX;
    cfg->scale = AG_SCALE;
    cfg->tab2  = ag_tab2;
    cfg->tab3  = ag_tab3;

    for (i = 0; i < ARM_GRAVITY_DOF; ++i) {
        cfg->fc[i]      = 0.0f;   /* 等辨识后再填 */
        cfg->fv[i]      = 0.0f;
        cfg->tau_max[i] = 8.0f;   /* 按电机额定留余量，按需改 */
    }

    cfg->slew_rate = 50.0f;       /* N·m/s */
    cfg->qd_eps    = 0.02f;       /* 约 1.1 °/s */
}

int ArmGravity_Init(ArmGravity_t *g, const ArmGravity_Cfg *cfg)
{
    int i;

    if ((g == NULL) || (cfg == NULL) || (cfg->tab2 == NULL) || (cfg->tab3 == NULL) ||
        (cfg->n < 2) || (cfg->q_max <= cfg->q_min) || (cfg->scale <= 0.0f)) {
        return -1;
    }

    g->cfg = *cfg;
    for (i = 0; i < ARM_GRAVITY_DOF; ++i) {
        g->tau_ff[i] = 0.0f;
    }
    g->enabled = 0;
    return 0;
}

void ArmGravity_Enable(ArmGravity_t *g, int on)
{
    int i;

    if (g == NULL) {
        return;
    }

    g->enabled = (uint8_t)(on ? 1 : 0);
    if (!g->enabled) {
        for (i = 0; i < ARM_GRAVITY_DOF; ++i) {
            g->tau_ff[i] = 0.0f;
        }
    }
}

void ArmGravity_Reset(ArmGravity_t *g)
{
    int i;

    if (g == NULL) {
        return;
    }

    for (i = 0; i < ARM_GRAVITY_DOF; ++i) {
        g->tau_ff[i] = 0.0f;
    }
}

void ArmGravity_Gravity(const ArmGravity_t *g, const float q[ARM_GRAVITY_DOF],
                        float grav[ARM_GRAVITY_DOF])
{
    int i;

    if ((g == NULL) || (q == NULL) || (grav == NULL)) {
        return;
    }

    for (i = 0; i < ARM_GRAVITY_DOF; ++i) {
        grav[i] = 0.0f;                       /* g1/g4/g5/g6 暂不补 */
    }

    grav[1] = lookup(&g->cfg, g->cfg.tab2, q[1], q[2]);
    grav[2] = lookup(&g->cfg, g->cfg.tab3, q[1], q[2]);
}

int ArmGravity_Update(ArmGravity_t *g, const float q[ARM_GRAVITY_DOF],
                      const float qd[ARM_GRAVITY_DOF], float dt,
                      float tau_ff[ARM_GRAVITY_DOF])
{
    float grav[ARM_GRAVITY_DOF];
    int i;

    if ((g == NULL) || (q == NULL) || (qd == NULL) || (tau_ff == NULL)) {
        return -1;
    }

    if (!g->enabled) {
        for (i = 0; i < ARM_GRAVITY_DOF; ++i) {
            tau_ff[i] = 0.0f;
            g->tau_ff[i] = 0.0f;
        }
        return 0;
    }

    ArmGravity_Gravity(g, q, grav);

    for (i = 0; i < ARM_GRAVITY_DOF; ++i) {
        const float w = qd[i];
        float target = grav[i];

        /* 摩擦：库仑（带死区）+ 粘滞 */
        if (w > g->cfg.qd_eps) {
            target += g->cfg.fc[i];
        } else if (w < -g->cfg.qd_eps) {
            target -= g->cfg.fc[i];
        }
        target += g->cfg.fv[i] * w;

        /* 限幅 */
        target = clampf(target, -g->cfg.tau_max[i], g->cfg.tau_max[i]);

        /* 斜率限制：避免上电/切换时的力矩阶跃 */
        if (dt > 0.0f) {
            const float dmax = g->cfg.slew_rate * dt;
            const float d = clampf(target - g->tau_ff[i], -dmax, dmax);
            g->tau_ff[i] += d;
        } else {
            g->tau_ff[i] = target;
        }

        tau_ff[i] = g->tau_ff[i];
    }

    return 0;
}

uint32_t ArmGravity_TableCrc(void)
{
    return (uint32_t)AG_CRC;
}
