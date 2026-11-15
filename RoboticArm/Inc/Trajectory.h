#ifndef __TRAJECTORY_H
#define __TRAJECTORY_H

typedef struct{
   float a0;
    float a1;
    float a2;
    float a3;
    float a4;
    float a5;
}PolyCoeffs_t;

typedef struct{
    float dt;//规划用时
    float v0,vf;//始末速度
    float a0,af;//始末加速度
}PolyConfig_t;


#endif /* __TRAJECTORY_H */ 
