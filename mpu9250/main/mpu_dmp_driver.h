/*
 * @Author: Zhenwei Song zhenwei.song@qq.com
 * @Date: 2024-02-28 18:53:28
 * @LastEditors: Zhenwei Song zhenwei.song@qq.com
 * @LastEditTime: 2024-02-29 15:49:48
 * @FilePath: \esp32_pdr\mpu9250\main\mpu_dmp_driver.h
 * @Description: 仅供学习交流使用
 * Copyright (c) 2024 by Zhenwei Song, All Rights Reserved.
 */
// #ifdef _MPU_DMP_DRIVER_H
// #define _MPU_DMP_DRIVER_H

#define MPU6500
#define MPU_I2C_SCL 22
#define MPU_I2C_SDA 21
#define DEFAULT_MPU_HZ (10) // 设置MPU9250的采样率

uint8_t mpu_init_i2c(void);

void gyro_data_ready_cb(void);

// 初始化mpu6050
uint8_t mpu_dmp_init(void);

// 获取数据，给外部调用
uint8_t dmp_get_data(void);

// #endif // DEBUG