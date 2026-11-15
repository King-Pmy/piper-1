#include "arm_kinematics.h"
#include <math.h>

const MHD_Param_t arm_params[6] = {
    /*alpha     a       d        offset    dir*/
    {0.0f,      0.0f,   0.07f,   0.0f,     1.0f},
    {M_PI/2.0f, 0.0f,   0.0f,    0.0f,     1.0f},
    {0.0f,     -0.3583f,0.0f,    M_PI/2.0f,1.0f},
    {M_PI/2.0f, 0.07f,  0.3583f, 0.0f,     1.0f},
    {-M_PI/2.0f,0.0f,   0.0f,    0.0f,     1.0f},
    {M_PI/2.0f, 0.0f,   0.0f,    0.0f,     1.0f},
};

/* cos(pitch) 小于这个值就按万向锁处理, 此时 roll 与 yaw 只有和/差确定 */
#define EULER_GIMBAL_EPS 1.0e-6f

/* 从旋转矩阵提取 ZYX 欧拉角, 只在正解内部用 */
static void MatrixToEulerZYX(const Matrix4x4 *T,float *roll,float *pitch,float *yaw){
    const float (*R)[4] = (const float (*)[4])T->data;
    float cp = sqrtf(R[0][0]*R[0][0] + R[1][0]*R[1][0]);

    *pitch = atan2f(-R[2][0], cp);

    if(cp > EULER_GIMBAL_EPS){
        *yaw  = atan2f(R[1][0], R[0][0]);
        *roll = atan2f(R[2][1], R[2][2]);
    }else{
        /* pitch = +-90 度, 约定 roll = 0, yaw 承担全部绕 z 的转动 */
        *roll = 0.0f;
        *yaw  = atan2f(-R[0][1], R[1][1]);
    }
}

void Arm_ForwardKinematics(const float q_motor[6],ArmPos_t *pose,Matrix4x4 *T_out){
    Matrix4x4 T_current,T_step;
    Matrix_Indentity4x4(&T_current);
    for(int i = 0;i<6;i++){
        float theta = q_motor[i] * arm_params[i].dir + arm_params[i].offset;

        float ct = cosf(theta);
        float st = sinf(theta);
        float ca = cosf(arm_params[i].alpha);
        float sa = sinf(arm_params[i].alpha);
        float a  = arm_params[i].a;
        float d  = arm_params[i].d;

        T_step.data[0][0] = ct;    T_step.data[0][1] = -st;   T_step.data[0][2] = 0;   T_step.data[0][3] = a;
        T_step.data[1][0] = st*ca; T_step.data[1][1] = ct*ca; T_step.data[1][2] = -sa; T_step.data[1][3] = -d*sa;
        T_step.data[2][0] = st*sa; T_step.data[2][1] = ct*sa; T_step.data[2][2] = ca;  T_step.data[2][3] = d*ca;
        T_step.data[3][0] = 0;     T_step.data[3][1] = 0;     T_step.data[3][2] = 0;   T_step.data[3][3] = 1;

        Matrix_Multiply4x4(&T_current,&T_step,&T_current);
    }

    if(pose){
        pose->x = T_current.data[0][3];
        pose->y = T_current.data[1][3];
        pose->z = T_current.data[2][3];
        MatrixToEulerZYX(&T_current, &pose->roll, &pose->pitch, &pose->yaw);
    }

    if(T_out){
        *T_out = T_current;
    }
}

/* 位姿 -> 4x4 齐次矩阵.
 * 欧拉角约定为 ZYX: R = Rz(yaw) * Ry(pitch) * Rx(roll).
 * 位置单位 m, 角度单位 rad. */
void Arm_PosToMatrix(const ArmPos_t *pos,Matrix4x4 *T_out){
    float cr, sr, cp, sp, cy, sy;

    if(!pos || !T_out){
        return;
    }

    cr = cosf(pos->roll);
    sr = sinf(pos->roll);
    cp = cosf(pos->pitch);
    sp = sinf(pos->pitch);
    cy = cosf(pos->yaw);
    sy = sinf(pos->yaw);

    T_out->data[0][0] = cy * cp;
    T_out->data[0][1] = cy * sp * sr - sy * cr;
    T_out->data[0][2] = cy * sp * cr + sy * sr;
    T_out->data[0][3] = pos->x;

    T_out->data[1][0] = sy * cp;
    T_out->data[1][1] = sy * sp * sr + cy * cr;
    T_out->data[1][2] = sy * sp * cr - cy * sr;
    T_out->data[1][3] = pos->y;

    T_out->data[2][0] = -sp;
    T_out->data[2][1] = cp * sr;
    T_out->data[2][2] = cp * cr;
    T_out->data[2][3] = pos->z;

    T_out->data[3][0] = 0.0f;
    T_out->data[3][1] = 0.0f;
    T_out->data[3][2] = 0.0f;
    T_out->data[3][3] = 1.0f;
}
