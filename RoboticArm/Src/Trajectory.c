#include "Trajectory.h"

void Trajectory_Calc(float q_start[6],float q_target[6],PolyCoeffs_t coeffs[6],PolyConfig_t config[6])
{
    for(int i=0;i<6;i++)
    {
        const float H = q_target[i] - q_start[i];
        const float dt = config[i].dt;
        if(config[i].v0 == 0.0f && config[i].vf == 0.0f && config[i].a0 == 0.0f && config[i].af == 0.0f)
        {
            coeffs[i].a0 = q_start[i];
            coeffs[i].a1 = 0;
            coeffs[i].a2 = 0;
            coeffs[i].a3 = (10 * H)/(dt*dt*dt);
            coeffs[i].a4 = (-15 * H)/(dt*dt*dt*dt);
            coeffs[i].a5 = (6 * H)/(dt*dt*dt*dt*dt);
        }else
        {
            const float v0 = config[i].v0;
            const float vf = config[i].vf;
            const float a0 = config[i].a0;
            const float af = config[i].af;
            coeffs[i].a0 = q_start[i];
            coeffs[i].a1 = v0;
            coeffs[i].a2 = a0/2;
            coeffs[i].a3 = (20.0f * H - (8.0f * vf + 12.0f * v0) * dt - (3.0f * a0 - af) * dt*dt) / (2.0f * dt*dt*dt);
            coeffs[i].a4 = (-30.0f * H + (14.0f * vf + 16.0f * v0) * dt + (3.0f * a0 - 2.0f * af) * dt*dt) / (2.0f * dt*dt*dt*dt);
            coeffs[i].a5 = (12.0f * H - 6.0f * (vf + v0) * dt - (a0 - af) * dt*dt) / (2.0f * dt*dt*dt*dt*dt);
        }
    }
}

float Trajectory_Eval(PolyCoeffs_t *coeffs,float t)
{
    float q = coeffs->a0 + coeffs->a1*t + coeffs->a2*t*t + (coeffs->a3+ (coeffs->a4 + coeffs->a5*t)*t)*t*t*t;
    return q;
}

/*   写法示例
    float q_start[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    float q_target[6] = {1.57f, -0.5f, 0.8f, 1.0f, 0.5f, 0.0f}; // 弧度
    
    PolyCoeffs_t coeffs[6];
    PolyConfig_t config[6];
    
    // 2. 配置参数 (全部设为 0，测试基础的 PTP 五次规划)
    for(int i = 0; i < 6; i++) {
        config[i].dt = 1.0f; // 1 秒走完
        config[i].v0 = 0.0f; config[i].vf = 0.0f;
        config[i].a0 = 0.0f; config[i].af = 0.0f;
    }
    
    // 3. 算系数
    Trajectory_Calc(q_start, q_target, coeffs, config);
    
    // 4. 开始平滑下发 (总耗时 1000ms，步长 1ms)
    for(int step = 0; step <= 1000; step++) {
        float t = step * 0.001f;
        float q_out[6];
        
        for(int i = 0; i < 6; i++) {
            q_out[i] = Trajectory_Eval(&coeffs[i], t);
        }
        
        // 下发到底层电机驱动
        // 假设你的函数叫 DM_Send_MIT，传入的是 6 个目标角度和速度
        // 注意：MIT 模式最好也传速度，你可以加一个 Trajectory_Eval_Velocity 来求导传进去
        DM_Send_MIT_Position(q_out); 
        
        // 阻塞延时 1ms (注意：这个只用于极简测试！)
        HAL_Delay(1); 
*/