/*
 * @Author: Zhenwei Song zhenwei.song@qq.com
 * @Date: 2024-03-06 16:01:13
 * @LastEditors: Zhenwei Song zhenwei.song@qq.com
 * @LastEditTime: 2024-03-07 12:54:40
 * @FilePath: \esp32_positioning\components\ins\inc\data_processing.h
 * @Description: 仅供学习交流使用
 * Copyright (c) 2024 by Zhenwei Song, All Rights Reserved.
 */
#ifndef __DATA_PROCESSING_H
#define __DATA_PROCESSING_H

#include "ins.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DATA_PROCESSING_DEBUG
#define PI 3.1415926

typedef struct quaternion {
    float w;
    float x;
    float y;
    float z;
} s_quaternion, *ps_quaternion;

typedef struct v {
    float x;
    float y;
    float z;
} s_v, *ps_v;

float deg_to_rad(float deg);

ps_point get_point(ps_point old_point, float dt, float gravity);

#ifdef __cplusplus
}
#endif
#endif