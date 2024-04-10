/*
 * @Author: Zhenwei Song zhenwei.song@qq.com
 * @Date: 2024-01-16 15:05:23
 * @LastEditors: Zhenwei Song zhenwei.song@qq.com
 * @LastEditTime: 2024-04-10 16:09:13
 * @FilePath: \esp32_positioning\components\yaw_init_by_mag\inc\yaw_init_by_mag.h
 * @Description: 仅供学习交流使用
 * Copyright (c) 2024 by Zhenwei Song, All Rights Reserved.
 */

#ifndef YAW_INIT_BY_MAG_H_
#define YAW_INIT_BY_MAG_H_

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_timer.h"

#include "./../../../main/main.h"

#include "./../../psins/inc/KFApp.h"
#include "./../../psins/inc/psins.h"

extern uint8_t imuDataAvailable;

extern float imu_acc[3];
extern float imu_mag[3];

extern CQuat qnb0;

extern const float MAG_uTPerLSB;
extern const float ACC_gPerLSB;

void AlignInit(uint8_t alignTime_s);

#endif