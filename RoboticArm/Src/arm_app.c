/**
  * @file    arm_app.c
  * @brief   机械臂应用层: 电机初始化、目标下发、接收分发.
  *
  * 控制任务每周期: Arm_Motor_Update() -> IsReady -> Arm_SettargetPos().
  */

#include "arm_app.h"
#include "arm_motor.h"
#include "arm_config.h"
#include "arm_kinematics.h"
#include "arm_ik.h"
#include "arm_gravity.h"
#include "arm_gravity_table.h"   
#include "Trajectory.h"
//#include "Curve_Following_3d.h"
#include "main.h"
#include "fdcan.h"
#include "cmsis_os2.h"
#include <stdbool.h>
#include <stdint.h>

#include "arm_config.h"

ArmMotorBus_t g_arm_bus;
ArmPos_t g_arm_pos;

const dm_motor_config_t test_cfg[5] = {
    /* 关节2 - 肩部俯仰 */
    { .device_id = 2, .master_id = 0x12, .mode = DM_MOTOR_MODE_MIT, .gear_ratio = 2.0f,
      .offline_ms = 300, .velocity_limit_radps = 15.0f, .invert_direction = false },
    /* 关节3 - 肘部俯仰 */
    { .device_id = 3, .master_id = 0x13, .mode = DM_MOTOR_MODE_MIT, .gear_ratio = 2.0f,
      .offline_ms = 300, .velocity_limit_radps = 15.0f, .invert_direction = false },
    /* 关节4 - 小臂旋转 */
    { .device_id = 4, .master_id = 0x14, .mode = DM_MOTOR_MODE_MIT, .gear_ratio = 1.0f,
      .offline_ms = 300, .velocity_limit_radps = 15.0f, .invert_direction = false },
    /* 关节5 - 腕部俯仰 */
    { .device_id = 5, .master_id = 0x15, .mode = DM_MOTOR_MODE_MIT, .gear_ratio = 1.0f,
      .offline_ms = 300, .velocity_limit_radps = 15.0f, .invert_direction = false },
    /* 关节6 - 腕部旋转 */
    { .device_id = 6, .master_id = 0x16, .mode = DM_MOTOR_MODE_MIT, .gear_ratio = 1.0f,
      .offline_ms = 300, .velocity_limit_radps = 15.0f, .invert_direction = false },
};  
#define JOINT_RATIO 2.0f   /* 电机 2 圈 = 关节 1 圈 */

static float motor_to_link_pos(float q_motor, float q_zero)
{
    return (q_motor - q_zero) / JOINT_RATIO;
}

static float link_to_motor_pos(float q_link, float q_zero)
{
    return q_zero + JOINT_RATIO * q_link;
}

static float motor_to_link_vel(float qd_motor)
{
    return qd_motor / JOINT_RATIO;
}

static float link_to_motor_vel(float dq_link)
{
    return JOINT_RATIO * dq_link;
}

static float link_to_motor_torque(float tau_link)
{
    return tau_link / JOINT_RATIO;   /* 忽略效率 */
}


volatile float q_motor[6]={0.0f,1.57f,0.58f,0.0f,0.0f,0.0f};//电机角
static float q_zero[6]={0.0f,-0.967f,-1.415f,0.0f,0.0f,0.0f};//电机零位
float q_link[6];//关节角
/*主任务*/
void StartMotorTask(void *argument)
{
  bool ok=ArmMotorBus_Init(&g_arm_bus,&hfdcan2,test_cfg ,5);
 if(ok){
    for(;;)
  {
    ArmMotorBus_Update(&g_arm_bus, HAL_GetTick());
    for(int i=0;i<6 ;i++){
    q_link[i] = (i==1||i==2)?motor_to_link_pos(q_motor[i],q_zero[i]):(q_motor[i]-q_zero[i]);
    }
    // Arm_SettestPos(q_motor);
    Arm_ForwardKinematics(q_link,&g_arm_pos,NULL);
    

    osDelay(5);
  }
 
  }
}

bool Arm_SettestPos(const volatile float pos[ARM_TEST])
{                                       //ARM_TEST在.h中改
    dm_motor_cmd_t cmds[ARM_TEST] = {0};

    
    for (uint8_t i = 0U; i < ARM_TEST; i++)
    {
        if (!ArmMotorBus_IsReady(&g_arm_bus, i))
        {
            return false;
        }
    }

    for (int i = 0; i < ARM_TEST; i++) {
        cmds[i].kind = test_cfg[i].mode;

        if (cmds[i].kind == DM_MOTOR_MODE_MIT) {
            cmds[i].u.mit.position_rad   = pos[i];
            cmds[i].u.mit.velocity_radps = 0.0f;
            cmds[i].u.mit.kp             = 5.0f;
            cmds[i].u.mit.kd             = 0.5f;//g_arm_gain.kd[i];
            cmds[i].u.mit.torque_n_m     = 0.0f;
        } 
        // else if (cmds[i].kind == DM_MOTOR_MODE_POSITION_VELOCITY) {
        //     cmds[i].u.position_velocity.position_rad   = pos[i];
        //     cmds[i].u.position_velocity.velocity_radps =
        //         arm_motor_configs[i].velocity_limit_radps;
        // } 
        else {
            return false;
        }
    }

    for (uint8_t i = 0U; i < ARM_TEST; i++){
     
           if (!ArmMotorBus_SetTargetOne(&g_arm_bus, i, &cmds[i]))
           {
              return false;
           }
       }

    return true;
}

//void Arm_Init(FDCAN_HandleTypeDef *hfdcan){
//    ArmMotorBus_Init(&g_arm_bus,hfdcan,arm_motor_configs,ARM_DOF);
//    ArmIK_Init(&g_ik,arm_params);
//    ArmIK_SetTool(&g_ik,0,0,0.083);
//    ArmGravity_DefaultCfg(&cfg);
//    ArmGravity_Init(&g_grav,&cfg);
//    ArmGravity_Enable(&g_grav,0);
//    // Trajectory_Init();

//}

//void Arm_Tick(void);//Update

bool Arm_SettargetPos(const float pos[ARM_DOF])
{
    dm_motor_cmd_t cmds[ARM_DOF] = {0};

    /* 6 轴全部就绪才动; 夹爪单独判断. */
    for (uint8_t i = 0U; i < ARM_JOINT_COUNT; i++)
    {
        if (!ArmMotorBus_IsReady(&g_arm_bus, i))
        {
            return false;
        }
    }

    for (int i = 0; i < ARM_DOF; i++) {
        cmds[i].kind = arm_motor_configs[i].mode;

        if (cmds[i].kind == DM_MOTOR_MODE_MIT) {
            cmds[i].u.mit.position_rad   = pos[i];
            cmds[i].u.mit.velocity_radps = 0.0f;
            cmds[i].u.mit.kp             = g_arm_gain.kp[i];
            cmds[i].u.mit.kd             = g_arm_gain.kd[i];
            cmds[i].u.mit.torque_n_m     = 0.0f;
        } else if (cmds[i].kind == DM_MOTOR_MODE_POSITION_VELOCITY) {
            cmds[i].u.position_velocity.position_rad   = pos[i];
            cmds[i].u.position_velocity.velocity_radps =
                arm_motor_configs[i].velocity_limit_radps;
        } else {
            return false;
        }
    }

    for (uint8_t i = 0U; i < ARM_DOF; i++)
    {
        if ((i == ARM_JOINT_COUNT) &&
            (!ArmMotorBus_IsReady(&g_arm_bus, i)))
        {
            continue;
        }

        if (!ArmMotorBus_SetTargetOne(&g_arm_bus, i, &cmds[i]))
        {
            return false;
        }
    }

    return true;
}



void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[64];

    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U) {
        return;
    }

    while (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK) {
        if ((rx_header.IdType != FDCAN_STANDARD_ID) || (rx_header.DataLength > 8U)) {
            continue;
        }

        ArmMotorBus_OnRx(&g_arm_bus, rx_header.Identifier, rx_data,
                         (uint8_t)rx_header.DataLength, HAL_GetTick());
    }
}
