#ifndef __ARM_KINEMATICS_H
#define __ARM_KINEMATICS_H


#include "arm_math.h"

#ifndef M_PI        
#define M_PI 3.14159265358979f
#endif  

typedef struct{
    float alpha;
    float a;
    float d;
    float offset;
    float dir;
}MHD_Param_t;

/* arm_params 定义在 arm_kinematics.c, 其他文件用 extern 引用 */
extern const MHD_Param_t arm_params[6];

/* 末端位姿: 位置单位 m, 欧拉角单位 rad.
   欧拉角顺序为 ZYX: R = Rz(yaw) * Ry(pitch) * Rx(roll),
   等价于 URDF/ROS 的 rpy (固定轴 x-y-z):
   roll 绕基座 x, [-pi, pi]; pitch 绕基座 y, [-pi/2, pi/2]; yaw 绕基座 z, [-pi, pi] */
typedef struct{
    float x;      /* m */
    float y;      /* m */
    float z;      /* m */
    float roll;   /* rad */
    float pitch;  /* rad */
    float yaw;    /* rad */
}ArmPos_t;

/* 位姿 -> 4x4 齐次矩阵, 行主序, 旋转在左上 3x3, 平移在最后一列 */
void Arm_PosToMatrix(const ArmPos_t *pos,Matrix4x4 *T_out);

/* 关节角 -> 位姿 (xyz + 欧拉角).
 * pose 和 T_out 都可以传 NULL: 只取位姿时 T_out 传 NULL,
 * 只取矩阵时 pose 传 NULL (ArmIK_Residual 用这种). */
void Arm_ForwardKinematics(const float q_link[6],ArmPos_t *pose,Matrix4x4 *T_out);

#endif
