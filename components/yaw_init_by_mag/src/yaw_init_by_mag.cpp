
/*
 * @Author: Zhenwei Song zhenwei.song@qq.com
 * @Date: 2024-01-16 15:05:32
 * @LastEditors: Zhenwei Song zhenwei.song@qq.com
 * @LastEditTime: 2024-04-10 16:14:37
 * @FilePath: \esp32_positioning\components\yaw_init_by_mag\src\yaw_init_by_mag.cpp
 * @Description: 仅供学习交流使用
 * Copyright (c) 2024 by Zhenwei Song, All Rights Reserved.
 */
#include "./../inc/yaw_init_by_mag.h"
#include "./../inc/calibration.h"

uint8_t imuDataAvailable = 0;

float imu_acc[3];
float imu_mag[3];
CQuat qnb0;
const double declination = 0.0; // 磁偏角(rad)

const float MAG_uTPerLSB = 9600.0f / (1 << 16);
const float ACC_gPerLSB = 8.0f / (1 << 16);

void AlignInit(uint8_t alignTime_s)
{
    int cnt = alignTime_s / my_TS, tmp = cnt;
    double accBuf[3] = {0.0}, magBuf[3] = {0.0};

    do {
        if (imuDataAvailable) {
            for (uint8_t i = 0; i < 3; i++) {
                accBuf[i] += imu_acc[i];
                magBuf[i] += imu_mag[i];
            }
            tmp--;
            imuDataAvailable = 0;
        }
    } while (tmp);

    CVect3 Acc = CVect3(accBuf) / cnt;
    CVect3 Mag = CVect3(magBuf) / cnt;
    IMURFU(&Mag, 1, "FRD");
    Mag = (Mag * 0.01 - b) * A;

    CVect3 att0 = O31;
    att0.i = asin(Acc.j);
    att0.j = -atan2(Acc.i, Acc.k);
    Mag = a2qua(att0) * Mag;
    double yaw = atan2Ex(Mag.i, Mag.j) + declination; // 可以直接调用老师的 magyaw函数
    if (yaw > PI)
        yaw = yaw - 2 * PI;
    else if (yaw < -PI)
        yaw = yaw + 2 * PI;
    att0.k = yaw;

    qnb0 = a2qua(att0);
    //	att0 /= DEG;				//调试用
}
