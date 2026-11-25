//#include "Curve_Following_3d.h"

//Curve_Following_3d_t circle_traj;
//Pose_t p1 = { .x = 300.0f,  .y = 0.0f, .z = 200.0f, .roll = 90 * DEG2RAD, .pitch = -90.0f * DEG2RAD, .yaw = 0 };
//Pose_t p2 = { .x = 150.0f, .y = -150.0f, .z = 250.0f, .roll = 0 * DEG2RAD, .pitch = 180.0f * DEG2RAD, .yaw = 0 };
//Pose_t p3 = { .x = 50.0f, .y = -150.0f,  .z = 170.0f, .roll = 0 * DEG2RAD, .pitch = 180.0f * DEG2RAD, .yaw = 0 };

//// --- 辅助宏：线性插值 ---
//#define LERP(a, b, s) ((a) + ((b) - (a)) * (s))

//void Curve_Following_3d_Init(Curve_Following_3d_t *traj, Curve_Following_3d_Type_t type, 
//                       float cx, float cy, float cz, 
//                       float ax, float ay, float az, 
//                       float roll, float pitch, float yaw, 
//                       float period_ms)
//{
//    traj->type = type;
//    traj->center_x = cx; traj->center_y = cy; traj->center_z = cz;
//    traj->amp_x = ax;    traj->amp_y = ay;    traj->amp_z = az;
//    traj->roll = roll;   traj->pitch = pitch; traj->yaw = yaw;
//    traj->period_ms = period_ms;
//    
//    traj->start_tick = HAL_GetTick();
//    traj->last_tick = traj->start_tick;
//    traj->time_seconds = 0.0f;
//    traj->is_running = 0; 
//    
//    traj->target_pose.x = cx; traj->target_pose.y = cy; traj->target_pose.z = cz;
//    traj->target_pose.roll = roll; traj->target_pose.pitch = pitch; traj->target_pose.yaw = yaw;
//}

//// ⭐ 新增：三点轨迹初始化
//void Curve_Following_3d_Init_3pt(Curve_Following_3d_t *traj, 
//                                 Pose_t p1, Pose_t p2, Pose_t p3, 
//                                 float ptp_time_s, float lin_time_s)
//{
//    traj->type = CURVE_3D_3PT;
//    traj->p1 = p1;
//    traj->p2 = p2;
//    traj->p3 = p3;
//    traj->ptp_time_s = ptp_time_s;
//    traj->lin_time_s = lin_time_s;
//    
//    traj->start_tick = HAL_GetTick();
//    traj->last_tick = traj->start_tick;
//    traj->seg_elapsed_time = 0.0f;
//    traj->is_running = 0;
//    traj->traj_3pt_state = TRAJ_3PT_IDLE;
//    
//    traj->target_pose = p1; // 初始姿态停在 P1
//	Curve_Following_3d_Start(&circle_traj);
//}

//void Curve_Following_3d_Start(Curve_Following_3d_t *traj)
//{
//    traj->start_tick = HAL_GetTick();
//    traj->last_tick = traj->start_tick;
//    traj->time_seconds = 0.0f;
//    traj->seg_elapsed_time = 0.0f;
//    traj->is_running = 1;
//    
//    if (traj->type == CURVE_3D_3PT) {
//        traj->traj_3pt_state = TRAJ_3PT_PTP; // 启动时直接进入第一段
//        traj->target_pose = traj->p1;
//    }
//}

//void Curve_Following_3d_Stop(Curve_Following_3d_t *traj)
//{
//    traj->is_running = 0;
//    if (traj->type == CURVE_3D_3PT) {
//        traj->traj_3pt_state = TRAJ_3PT_IDLE;
//    }
//}

//// ⭐ 带状态记忆的智能轨迹 API
//Pose_t Curve_Following_3d_SmartTarget(Curve_Following_3d_t *traj, uint32_t approach_delay_ms) 
//{
//    static uint32_t approach_tick = 0;
//    static uint8_t phase = 0; // 0:飞向P1, 1:跑轨迹, 2:已停止/完成

//    // 如果处于停止或完成状态，原地冻结，绝不乱动！
//    if (phase == 2) {
//        return traj->target_pose; 
//    }

//    // --- 阶段 0：飞向 P1 并等待 ---
//    if (phase == 0) {
//        if (approach_tick == 0) {
//            approach_tick = HAL_GetTick(); // 首次调用，记录时间
//            return traj->p1;
//        }
//        if (HAL_GetTick() - approach_tick < approach_delay_ms) {
//            return traj->p1; // 还没到时间，继续强制飞向 P1
//        }
//        // 延时结束，正式启动规划器！
//        Curve_Following_3d_Start(traj);
//        phase = 1; 
//    }

//    // --- 阶段 1：正式跑轨迹 ---
//    if (phase == 1) {
//        // ⚠️ 核心防呆：如果你在这个阶段调用了 Curve_Following_3d_Stop
//        // 库里的 is_running 会变成 0。此时立刻锁死相位，保命！
//        if (traj->is_running == 0) {
//            phase = 2; 
//            return traj->target_pose; // 返回停止瞬间的位置，不再更新
//        }
//        
//        Curve_Following_3d_Update(traj);
//    }

//    return traj->target_pose;
//}

//int Curve_Following_3d_Update(Curve_Following_3d_t *traj)
//{
//    if (!traj->is_running) return 0;

//    uint32_t now = HAL_GetTick();
//    float dt = 0.0f;
//    if (now != traj->last_tick) {
//        dt = (float)(now - traj->last_tick) / 1000.0f;
//        traj->time_seconds += dt;
//        traj->last_tick = now;
//    }

//    float period_s = traj->period_ms / 1000.0f;
//    float omega = 2.0f * PI / period_s; 
//    float t = traj->time_seconds;
//    float progress; 
//    
//    // ================== 原有曲线逻辑 ==================
//    switch (traj->type) {
//        case CURVE_3D_CIRCLE:
//            traj->target_pose.x = traj->center_x + traj->amp_x * cosf(omega * t);
//            traj->target_pose.y = traj->center_y + traj->amp_y * sinf(omega * t);
//            traj->target_pose.z = traj->center_z;
//            traj->target_pose.roll = traj->roll; traj->target_pose.pitch = traj->pitch; traj->target_pose.yaw = traj->yaw;
//            break;
//            
//        case CURVE_3D_LINE:
//            traj->target_pose.x = traj->center_x + traj->amp_x * sinf(omega * t);
//            traj->target_pose.y = traj->center_y;
//            traj->target_pose.z = traj->center_z;
//            traj->target_pose.roll = traj->roll; traj->target_pose.pitch = traj->pitch; traj->target_pose.yaw = traj->yaw;
//            break;
//            
//        case CURVE_3D_FIGURE_8:
//            traj->target_pose.x = traj->center_x + traj->amp_x * sinf(omega * t);
//            traj->target_pose.y = traj->center_y + traj->amp_y * sinf(2.0f * omega * t);
//            traj->target_pose.z = traj->center_z;
//            traj->target_pose.roll = traj->roll; traj->target_pose.pitch = traj->pitch; traj->target_pose.yaw = traj->yaw;
//            break;
//            
//        case CURVE_3D_SPIRAL:
//            progress = fmodf(t / period_s, 1.0f);
//            traj->target_pose.x = traj->center_x + traj->amp_x * cosf(omega * t);
//            traj->target_pose.y = traj->center_y + traj->amp_y * sinf(omega * t);
//            traj->target_pose.z = traj->center_z + traj->amp_z * progress;
//            traj->target_pose.roll = traj->roll; traj->target_pose.pitch = traj->pitch; traj->target_pose.yaw = traj->yaw;
//            break;

//        // ================== ⭐ 新增：三点轨迹状态机 ==================
//        case CURVE_3D_3PT:
//            traj->seg_elapsed_time += dt;
//            
//            float t1 = traj->ptp_time_s;         // 第一段耗时 (P1 -> P2)
//            float t_wait = 2.0f;                 // ⭐ 在点2等待的时间，这里设为 2 秒，你可以随意改
//            float t2 = traj->lin_time_s;         // 第二段耗时 (P2 -> P3)
//            float total_time = t1 + t_wait + t2; // 总耗时
//            
//            if (traj->seg_elapsed_time <= t1) {
//                // --- 阶段 1：P1 到 P2 ---
//                float s = traj->seg_elapsed_time / t1;
//                traj->target_pose.x = LERP(traj->p1.x, traj->p2.x, s);
//                traj->target_pose.y = LERP(traj->p1.y, traj->p2.y, s);
//                traj->target_pose.z = LERP(traj->p1.z, traj->p2.z, s);
//                traj->target_pose.roll  = LERP(traj->p1.roll,  traj->p2.roll,  s);
//                traj->target_pose.pitch = LERP(traj->p1.pitch, traj->p2.pitch, s);
//                traj->target_pose.yaw   = LERP(traj->p1.yaw,   traj->p2.yaw,   s);
//            } 
//            else if (traj->seg_elapsed_time <= (t1 + t_wait)) {
//                // --- 阶段 2：⭐ 在 P2 点原地等待（时间流逝，但位姿不变） ---
//                traj->target_pose = traj->p2;
//            }
//            else if (traj->seg_elapsed_time <= total_time) {
//                // --- 阶段 3：P2 到 P3 （斜向直线推入） ---
//                float s = (traj->seg_elapsed_time - t1 - t_wait) / t2;
//                traj->target_pose.x = LERP(traj->p2.x, traj->p3.x, s);
//                traj->target_pose.y = LERP(traj->p2.y, traj->p3.y, s);
//                traj->target_pose.z = LERP(traj->p2.z, traj->p3.z, s);
//                
//                // ⚠️ 姿态绝对锁定为 P2 的值
//                traj->target_pose.roll  = traj->p2.roll;
//                traj->target_pose.pitch = traj->p2.pitch;
//                traj->target_pose.yaw   = traj->p2.yaw;
//            } 
//            else {
//                // --- 阶段 4：轨迹走完，锁定在 P3 ---
//                 traj->target_pose = traj->p3;
//                //traj->is_running = 0; // 如果想让机械臂自动停下，取消这行注释
//            }
//            break;

//        default:
//            break;
//    }

//    return 1;
//}