//#ifndef __CURVE_FOLLOWING_3D_H__
//#define __CURVE_FOLLOWING_3D_H__

//#include "main.h"
//#include <math.h>
//#include "piper_kinematics.h" // 引入你的逆解库

//// 三维轨迹类型
//typedef enum {
//    CURVE_3D_CIRCLE,    // 空间圆
//    CURVE_3D_LINE,      // 空间直线往返
//    CURVE_3D_FIGURE_8,  // 空间 8 字型
//    CURVE_3D_SPIRAL,    // 空间螺旋线
//    CURVE_3D_3PT        // ⭐ 新增：三点轨迹（先PTP平滑靠近，再直线斜插锁定姿态）
//} Curve_Following_3d_Type_t;

//// 三点轨迹运行状态机
//typedef enum {
//    TRAJ_3PT_IDLE = 0,
//    TRAJ_3PT_PTP,       // 第一段：P1 到 P2 平滑过渡
//    TRAJ_3PT_LIN,       // 第二段：P2 到 P3 斜向直线，姿态锁定
//    TRAJ_3PT_DONE       // 完成
//} Traj_3pt_State_t;

//// 三维轨迹结构体
//typedef struct {
//    Curve_Following_3d_Type_t type;  // 轨迹类型
//    
//    // 几何参数 (用于圆弧、螺旋等)
//    float center_x, center_y, center_z; 
//    float amp_x, amp_y, amp_z;          
//    
//    // 姿态参数 (用于圆弧、螺旋等)
//    float roll, pitch, yaw;             
//    
//    float period_ms;      // 运行周期 (ms)
//    
//    // ⭐ 新增：三点轨迹专用参数
//    Pose_t p1, p2, p3;       // 三个目标点
//    float ptp_time_s;        // 第一段耗时 (秒)
//    float lin_time_s;        // 第二段耗时 (秒)
//    float seg_elapsed_time;  // 当前段已耗时
//    Traj_3pt_State_t traj_3pt_state; // 三点状态机

//    // 运行时状态
//    uint32_t start_tick;
//    uint32_t last_tick;
//    float    time_seconds;
//    uint8_t  is_running; 
//    
//    Pose_t   target_pose; // 当前计算出的三维位姿
//} Curve_Following_3d_t;

//extern Curve_Following_3d_t circle_traj;

//// API
  
//void Curve_Following_3d_Init(Curve_Following_3d_t *traj, Curve_Following_3d_Type_t type, 
//                       float cx, float cy, float cz, 
//                       float ax, float ay, float az, 
//                       float roll, float pitch, float yaw, 
//                       float period_ms);

//// ⭐ 新增 API：专门用于初始化三点轨迹
//extern Pose_t p1;
//extern Pose_t p2;					   
//extern Pose_t p3;  		   
//void Curve_Following_3d_Init_3pt(Curve_Following_3d_t *traj, 
//                                 Pose_t p1, Pose_t p2, Pose_t p3, 
//                                 float ptp_time_s, float lin_time_s);

//void Curve_Following_3d_Start(Curve_Following_3d_t *traj);
//void Curve_Following_3d_Stop(Curve_Following_3d_t *traj);
//Pose_t Curve_Following_3d_SmartTarget(Curve_Following_3d_t *traj, uint32_t approach_delay_ms);
//// 更新三维坐标
//int Curve_Following_3d_Update(Curve_Following_3d_t *traj);

//#endif