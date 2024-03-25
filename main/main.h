/*
 * @Author: Zhenwei Song zhenwei.song@qq.com
 * @Date: 2024-03-11 15:46:52
 * @LastEditors: Zhenwei Song zhenwei.song@qq.com
 * @LastEditTime: 2024-03-25 16:17:22
 * @FilePath: \esp32_positioning\main\main.h
 * @Description: 仅供学习交流使用
 * Copyright (c) 2024 by Zhenwei Song, All Rights Reserved.
 */
#ifndef __MAIN_H
#define __MAIN_H

#include "./../components/mpu9250/inc/range.h"
#include "./../components/psins/inc/mcu_init.h"

// #define DEBUG

/* -------------------------------------------------------------------------- */
/*                                  数据读取方式设定                                  */
/* -------------------------------------------------------------------------- */
// #define USING_DMP//I2C读DMP
#define USING_REG // I2C直接读寄存器

#if defined USING_DMP || defined USING_REG
#define USING_I2C
#endif

#if !defined USING_I2C && !defined USING_SPI
#define USING_SPI
#endif
/* -------------------------------------------------------------------------- */
/*                                  数据解析方式设定                                  */
/* -------------------------------------------------------------------------- */
//  #define PSINS_ATT //仅获取姿态角
#define PSINS_POS // 获取位置(包括姿态)

// #define USING_SFANN_SINS // 用简化的捷联惯导定位

#ifdef USING_I2C

#define GET_RAW_INFO // 从寄存器直接获取角速度和加速度（和SPI一样）
#define I2C_SCL 22
#define I2C_SDA 21
#define GPIO_INTR 23
#define DEFAULT_HZ (10)     // 设置MPU9250的采样率
#define A_RANGE A_RANGE_4   // 传感器加速度量程
#define G_RANGE G_RANGE_500 // 传感器加速度量程

#define LONGITUDE 113.572256 // 经度
#define LATITUDE 23.305836   // 纬度
#define ALTITUDE 59          // 海拔

#define my_TS 1.0 / DEFAULT_HZ
#define SAMPLE_RATE DEFAULT_HZ

#define GET_LINEAR_ACC_AND_G // 获取除去重力的线性加速度
#endif

#if (A_RANGE == A_RANGE_2)
#define A_RANGE_NUM A_RANGE_2_NUM
#elif (A_RANGE == A_RANGE_4)
#define A_RANGE_NUM A_RANGE_4_NUM
#elif (A_RANGE == A_RANGE_8)
#define A_RANGE_NUM A_RANGE_8_NUM
#elif (A_RANGE == A_RANGE_16)
#define A_RANGE_NUM A_RANGE_16_NUM
#endif // A_RANGE

#if defined PSINS_ATT || defined PSINS_POS
#define USING_PSINS
#endif

#if !defined USING_PSINS && !defined USING_INS && !defined USING_SFANN_SINS
#define USING_INS
#endif

#if !defined USING_DMP && !defined USING_RAW
#define USING_RAW // 用直接读寄存器的方式
#endif

#ifdef USING_SPI
#warning Using spi!!!!!!
#endif

#ifdef USING_INS
#warning Using ins!!!!!!
#endif

#ifdef USING_RAW
#warning Using raw!!!!!!
#endif

#if defined USING_I2C && defined USING_SPI
#error "Can't use I2C and SPI at the same time!"
#endif

#if defined USING_PSINS && defined USING_INS
#error "Can't use PSIN and INS at the same time!"
#endif

#if defined USING_RAW && defined USING_DMP
#error "Can't use RAW and DMP at the same time!"
#endif

extern MPU_AD_value mpu_AD_value;
extern MPU_Data_value mpu_Data_value;
extern GPS_Data_value gps_Data_value;
extern INS_Data_value ins_Data_value;
extern Out_Data out_data;

extern uint8_t MS5611_cnt;

extern uint8_t mcu_init_gpscfg;
extern uint8_t Usart1_out_DATA[200];
extern uint8_t Usart1_out_Length;

extern uint8_t Rx2_data[120], Rx2_data1[120];
extern uint8_t Rx2_complete;
extern uint16_t Length2;

extern uint8_t PPs_cnt;
extern uint8_t GPS_exist;
extern uint16_t GPS_break_cnt;

extern uint32_t OUT_cnt;
extern uint32_t GPS_Delay;

extern uint8_t GAMT_OK_flag;
extern uint8_t GPS_OK_flag;
extern uint8_t Bar_OK_flag;

#endif /* __MAIN_H */
