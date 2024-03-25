#ifndef __INS_H
#define __INS_H

#define G 9.8

typedef struct point {
    float position[3];
    float speed[3];
    float q[4];          // 四元数
    float acc[3];        // 加速度
    float linear_acc[3]; // 加速度
    float gyr[3];        // 角速度
    float acc_raw[3];    // 加速度
    float gyr_raw[3];    // 角速度
    short gyr_fifo[3];
    short acc_fifo[3];
} s_point, *ps_point;

#endif