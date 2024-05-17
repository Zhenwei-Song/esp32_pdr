/*
 * @Author: Zhenwei Song zhenwei.song@qq.com
 * @Date: 2024-03-25 15:36:20
 * @LastEditors: Zhenwei Song zhenwei.song@qq.com
 * @LastEditTime: 2024-05-13 12:01:37
 * @FilePath: \esp32_positioning\main\main.cpp
 * @Description: 仅供学习交流使用
 * Copyright (c) 2024 by Zhenwei Song, All Rights Reserved.
 * @Author: Zhenwei Song zhenwei.song@qq.com
 * @Date: 2024-02-28 18:53:28
 * @LastEditors: Zhenwei Song zhenwei.song@qq.com
 * @LastEditTime: 2024-03-19 16:40:02
 * @FilePath: \esp32_positioning\main\main.cpp
 * 驱动mpu9250，串口输出欧拉角，可用上位机进行串口连接查看图像
 * 利用官方dmp库输出欧拉角（使用I2C时）
 * 使用I2C连接,或者SPI(SPI存在陀螺仪z轴无数据情况，且速度慢，不推荐)
 * 引脚使用（I2C）：INT 23 SCL 22  SDA 21 供电3.3v
 * 引脚使用（SPI）: SCL 25 SDA 33 AD0 32 NCS 26
 * 采样率设置(I2C)：最快为200Hz(5ms采集一次)
 * 加速度量程设置：2到16g
 * 陀螺仪量程设置：250到2000deg/s
 * 读取mpu9250数据可采用dmp（i2c，磁力无法读），直接读寄存器（i2c），直接读寄存器（spi，存在数据读不全，速度慢问题）
 * 姿态解析可采用DMP（无磁力信息）、PSINS中解析，以及简化版捷联惯导sfann_sins(未验证)
 * psins可以使用串口上传数据，由电脑端运行psins进行解析
 *坐标广研院：23.305836,113.572256，59
 * @Description: 仅供学习交流使用
 * Copyright (c) 2024 by Zhenwei Song, All Rights Reserved.
 */
#include "./main.h"
#include "./all_tasks.h"

#include "./../components/mpu9250/inc/empl_driver.h"
#include "./../components/mpu_timer/inc/positioning_timer.h"

#ifdef USING_DMP
#include "./../components/mpu9250/inc/inv_mpu.h"
#include "./../components/mpu9250/inc/mpu_dmp_driver.h"
#endif // USING_DMP

#ifdef USING_RAW
#include "./../components/get_acc_without_g/inc/get_acc_without_g.h"
#include "./../components/mpu9250/inc/mpu9250_raw.h"
#endif // USING_RAW

#ifdef USING_SPI
#include "./../components/psins/inc/mcu_init.h"
#endif

#ifdef USING_INS
#include "./../components/ins/inc/data_processing.h"
#include "./../components/ins/inc/ins.h"
#endif // USING_INS

#ifdef USING_PSINS
#include "./../components/mpu9250/inc/mpu_dmp_driver.h"
#include "./../components/psins/inc/mcu_init.h"
#include "./../components/psins/inc/uart_out.h"
#endif // USING_PSINS

#ifdef USING_SFANN_SINS
#include "./../components/mpu9250/inc/mpu_dmp_driver.h"
#endif // USING_SFANN_SINS

#ifdef USING_PSINS
#include "./../components/mpu9250/inc/mpu_dmp_driver.h"
#include "./../components/psins/inc/KFApp.h"
#ifdef PSINS_UART
#include "./../components/psins/inc/uart_out.h"
#include "my_uart.h"
#endif
#endif // USING_PSINS

#include <stdio.h>
#include <stdlib.h>

// static CKFApp kf(my_TS);

extern "C" void app_main(void)
{
/* -------------------------------------------------------------------------- */
/*                                   IMU数据获取                                  */
/* -------------------------------------------------------------------------- */
#ifdef USING_DMP
    positioning_timer_init();
    mpu_dmp_init();
    // initAK8963_2(magCalibration);
    i2c_gpio_init();
    ins_init();
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    start_i2c_isr();
    xTaskCreate(get_dmp_data, "get_dmp_data", 4096, NULL, 10, NULL);
#endif // USING_DMP

#ifdef USING_RAW
    uint8_t res = 0;
    mpu_init_i2c();
    res = RAW_MPU9250_Init();
    if (res != 0) {
        printf("raw_mPU9250_init() failed %d\n", res);
    }
    positioning_timer_init();
    esp_timer_start_periodic(positioning_time3_timer, TIME3_TIMER_PERIOD);
    printf("check point 0\n");
    xCountingSemaphore_timeout3 = xSemaphoreCreateCounting(200, 0);
    xCountingSemaphore_push_data = xSemaphoreCreateCounting(200, 0);
    xTaskCreatePinnedToCore(get_raw_data_i2c, "get_raw_data_i2c", 4096, NULL, 10, NULL, 1);
    xTaskCreatePinnedToCore(push_raw_data, "push_raw_data", 4096, NULL, 8, NULL, 1);
#endif // USING_RAW

#ifdef USING_SPI
    xCountingSemaphore_timeout1 = xSemaphoreCreateCounting(200, 0);
    mcu_init();
    xTaskCreate(get_raw_data_spi, "get_raw_data_spi", 4096, NULL, 4, NULL);
#endif // USING_SPI
/* -------------------------------------------------------------------------- */
/*                                    数据解析                                    */
/* -------------------------------------------------------------------------- */
#ifdef PSINS_ATT
    xCountingSemaphore_data_update = xSemaphoreCreateCounting(200, 0);
    xTaskCreate(psins_att_data_update, "psins_att_data_update", 4096, NULL, 4, NULL);
#endif // PSINS_ATT

#ifdef PSINS_POS
#ifdef PSINS_UART
    esp_timer_start_periodic(positioning_time1_timer, TIME1_TIMER_PERIOD);
    xCountingSemaphore_timeout2 = xSemaphoreCreateCounting(200, 0);
    xTaskCreatePinnedToCore(psins_uart_pop_data, "psins_uart_pop_data", 8129, NULL, 7, NULL, 0);
    esp_timer_start_periodic(positioning_time2_timer, TIME2_TIMER_PERIOD);
#endif // PSINS_UART

    xCountingSemaphore_data_update = xSemaphoreCreateCounting(200, 0);
#if 0
    xTaskCreatePinnedToCore(psins_static_pos_data_update, "psins_static_pos_data_update", 15360, NULL, 9, NULL, 1);
#else

    psins_uart_init();
/* -------------------------------------------------------------------------- */
/*                             初始校准+SINSGNSS截取SINS                            */
/* -------------------------------------------------------------------------- */
#if 0
    bool align_ok = false;
    bool gyro_zero_bias_cal_ok = false;
    int bias_count = 0;
    float temp_gyro_bias[3] = {0};
    float temp_acc_bias[3] = {0};

    float temp_acc_without_g[3] = {0};
    float temp_pitch;
    float temp_roll;

    float my_vm[3] = {0};

    // int time_counter = 100;

    CVect3 pos0 = LLH(LATITUDE, LONGITUDE, ALTITUDE);
    CAligni0 align(pos0);
    CKFApp kf(my_TS);
    // CKFApp kf_temp(my_TS);
    bool kf_temp_initialized = false;
    bool acc_zero_bias_cal_ok = false;
    double yaw0 = C360CC180(0 * glv.deg); // 北偏东为正
    // kf.Init(CSINS(a2qua(CVect3(0, 0, yaw0)), O31, gpspos)); // 请正确初始化方位和位置
    CVect3 db = O31;
    CVect3 eb = O31;
    // vector3D linearAcc = {0, 0, 0};
    // vector3D gravity = {0.0, 0.0, -(float)glv.g0};
    // eulerAngles angles = {0, 0, 0};
    // vector3D_G acc_result;
    printf("Initalizing!!\n");
    while (1) {
        if (xSemaphoreTake(xCountingSemaphore_data_update, portMAX_DELAY) == pdTRUE) {

            // mpu_Data_value.Accel[0] = 0;
            // mpu_Data_value.Accel[1] = 0;
            // mpu_Data_value.Accel[2] = 1;
            if (gyro_zero_bias_cal_ok == false) {
                for (int i = 0; i < 3; i++) {
                    temp_gyro_bias[i] = temp_gyro_bias[i] + mpu_Data_value.Gyro[i];
                    // temp_acc_bias[i] = temp_acc_bias[i] + mpu_Data_value.Accel[i];
                }
                // temp_roll = atan2(mpu_Data_value.Accel[1], mpu_Data_value.Accel[2]);
                // temp_pitch = -atan2(mpu_Data_value.Accel[0], sqrt(mpu_Data_value.Accel[1] * mpu_Data_value.Accel[1] + mpu_Data_value.Accel[2] * mpu_Data_value.Accel[2]));
                // temp_acc_without_g[0] = mpu_Data_value.Accel[1] - sin(temp_roll) * cos(temp_pitch);
                // temp_acc_without_g[1] = mpu_Data_value.Accel[2] - cos(temp_roll) * cos(temp_pitch);
                // temp_acc_without_g[2] = mpu_Data_value.Accel[0] + sin(temp_pitch);
                // printf("temp roll pitch: (%f, %f)\n", temp_roll, temp_pitch);
                // printf("temp_acc_without_g: (%f, %f, %f)\n", temp_acc_without_g[0], temp_acc_without_g[1], temp_acc_without_g[2]);
                // printf("temp_acc_g: (%f, %f, %f)\n", sin(temp_roll) * cos(temp_pitch) * glv.g0, cos(temp_roll) * cos(temp_pitch) * glv.g0, -sin(temp_pitch) * glv.g0);
                temp_acc_bias[0] = temp_acc_bias[0] + mpu_Data_value.Accel[0];
                temp_acc_bias[1] = temp_acc_bias[1] + mpu_Data_value.Accel[1];
                temp_acc_bias[2] = temp_acc_bias[2] + (mpu_Data_value.Accel[2] - 1);
                // temp_acc_bias[0] = temp_acc_bias[0] + temp_acc_without_g[0];
                // temp_acc_bias[1] = temp_acc_bias[1] + temp_acc_without_g[1];
                // temp_acc_bias[2] = temp_acc_bias[2] + temp_acc_without_g[2];
                bias_count = bias_count + 1;
                if (OUT_cnt > ZERO_BIAS_CAL_TIME) {
                    gyro_zero_bias_cal_ok = true;
                    for (int i = 0; i < 3; i++) {
                        temp_gyro_bias[i] = temp_gyro_bias[i] / bias_count;
                        temp_acc_bias[i] = temp_acc_bias[i] / bias_count;
                        printf("temp_gyro_bias[%d] %f\n", i, temp_gyro_bias[i]);
                        printf("temp_acc_bias[%d] %f \n", i, temp_acc_bias[i]);
                    }
                    eb = CVect3(temp_gyro_bias[0], temp_gyro_bias[1], temp_gyro_bias[2]) * glv.dps; // 陀螺零偏 deg/s
                    db = CVect3(temp_acc_bias[0], temp_acc_bias[1], temp_acc_bias[2]) * glv.g0;
                    printf("Finish calculating bias!!\n");
                }
            }
            else {
                // if (acc_zero_bias_cal_ok == false) {
                //     CVect3 wm_temp = (*(CVect3 *)mpu_Data_value.Gyro * glv.dps - eb) * my_TS;
                //     CVect3 vm_temp = (*(CVect3 *)mpu_Data_value.Accel * glv.g0 - db) * my_TS;
                //     if (kf_temp_initialized == false) {
                //         kf.Init(CSINS(a2qua(CVect3(0, 0, yaw0)), O31, pos0));
                //         kf_temp_initialized = true;
                //         temp_acc_bias[0] = 0;
                //         temp_acc_bias[1] = 0;
                //         temp_acc_bias[2] = 0;
                //     }
                //     else {
                //         kf.Update(&wm_temp, &vm_temp, 1, my_TS, 5);
                //         printf("kf_temp.sins.att: %f,%f,%f\n", kf.sins.att.i / DEG, kf.sins.att.j / DEG, kf.sins.att.k / DEG);
                //         linearAcc.x = mpu_Data_value.Accel[0] * glv.g0;
                //         linearAcc.y = mpu_Data_value.Accel[1] * glv.g0;
                //         linearAcc.z = mpu_Data_value.Accel[2] * glv.g0;
                //         angles.pitch = kf.sins.att.i / DEG;
                //         angles.roll = kf.sins.att.j / DEG;
                //         angles.yaw = kf.sins.att.k / DEG;
                //         vector3D_G acc_result = get_acc_without_g(linearAcc, gravity, angles);
                //         printf("acceleration without G: %f, %f, %f\n", acc_result.x / glv.g0, acc_result.y / glv.g0, acc_result.z / glv.g0);
                //         printf("Gravity Acceleration: (%f, %f, %f)\n", acc_result.Gx / glv.g0, acc_result.Gy / glv.g0, acc_result.Gz / glv.g0);
                //         temp_acc_bias[0] = temp_acc_bias[0] + (mpu_Data_value.Accel[0] - 1 * acc_result.x / glv.g0);
                //         temp_acc_bias[1] = temp_acc_bias[1] + (mpu_Data_value.Accel[1] - 1 * acc_result.y / glv.g0);
                //         temp_acc_bias[2] = temp_acc_bias[2] + (mpu_Data_value.Accel[2] - 1 * acc_result.z / glv.g0);
                //         printf("temp_acc_bias_adjusted %f, %f, %f \n", temp_acc_bias[0], temp_acc_bias[1], temp_acc_bias[2]);
                //         acc_zero_bias_cal_ok = true;
                //     }
                // }
                if (0) {
                }
                else {
                    // linearAcc.x = mpu_Data_value.Accel[0] * glv.g0;
                    // linearAcc.y = mpu_Data_value.Accel[1] * glv.g0;
                    // linearAcc.z = mpu_Data_value.Accel[2] * glv.g0;
                    // angles.pitch = (kf.sins.att.i / DEG);
                    // angles.roll = (kf.sins.att.j / DEG);
                    // angles.yaw = (kf.sins.att.k / DEG);
                    // acc_result = get_acc_without_g(linearAcc, gravity, angles);
                    // printf("acceleration without G: %f, %f, %f\n", acc_result.x, acc_result.y, acc_result.z);
                    // printf("Gravity Acceleration: (%f, %f, %f)\n", acc_result.Gx, acc_result.Gy, acc_result.Gz);
                    // printf("Gravity Acceleration 2: (%f, %f, %f)\n", acc_result.Gx / glv.g0, acc_result.Gy / glv.g0, acc_result.Gz / glv.g0);
                    // db = CVect3((acc_result.Gx / glv.g0), (acc_result.Gy / glv.g0), (acc_result.Gz / glv.g0)) * glv.g0;

                    // temp_roll = atan2(mpu_Data_value.Accel[1], mpu_Data_value.Accel[2]);
                    // temp_pitch = -atan2(mpu_Data_value.Accel[0], sqrt(mpu_Data_value.Accel[1] * mpu_Data_value.Accel[1] + mpu_Data_value.Accel[2] * mpu_Data_value.Accel[2]));
                    // temp_acc_without_g[0] = mpu_Data_value.Accel[1] * glv.g0 - sin(temp_roll) * cos(temp_pitch) * glv.g0;
                    // temp_acc_without_g[1] = mpu_Data_value.Accel[2] * glv.g0 - cos(temp_roll) * cos(temp_pitch) * glv.g0;
                    // temp_acc_without_g[2] = mpu_Data_value.Accel[0] * glv.g0 + sin(temp_pitch) * glv.g0;
                    // printf("temp_acc_without_g: (%f, %f, %f)\n", temp_acc_without_g[0], temp_acc_without_g[1], temp_acc_without_g[2]);
                    // //printf("temp_acc_g: (%f, %f, %f)\n", sin(temp_roll) * cos(temp_pitch) * glv.g0, cos(temp_roll) * cos(temp_pitch) * glv.g0, -sin(temp_pitch) * glv.g0);
                    CVect3 wm = (*(CVect3 *)mpu_Data_value.Gyro * glv.dps - eb) * my_TS;
                    CVect3 vm = (*(CVect3 *)mpu_Data_value.Accel * glv.g0 - db) * my_TS;
                    // CVect3 vm = (*(CVect3 *)mpu_Data_value.Accel * glv.g0) * my_TS;
                    //  printf("my_v: (%f,%f,%f)\n", mpu_Data_value.Accel[0] * my_TS * glv.g0, mpu_Data_value.Accel[1] * my_TS * glv.g0, mpu_Data_value.Accel[2] * my_TS * glv.g0);
                    //   kf.Update(&wm, &vm, 1, my_TS, 5);
                    //   AVPUartOut(kf);

                    if (align_ok == false) {
                        align.Update(&wm, &vm, 1, my_TS, O31);
                        if (OUT_cnt > (ALIGN_TIME + ZERO_BIAS_CAL_TIME)) {
                            align_ok = true;
                            kf.Init(CSINS(a2qua(CVect3(0, 0, yaw0)), O31, pos0)); // 请正确初始化方位和位置
                            printf("Finish Aligning!!\n");
                        }
                    }
                    else {
                        kf.Update(&wm, &vm, 1, my_TS, 5);
#if 0
                        for (int i = 0; i < 3; i++) {
                            // my_vm[i] = (mpu_Data_value.Accel[i] * glv.g0 - temp_acc_bias[i] * glv.g0) * my_TS;
                            // my_vm[i] = (mpu_Data_value.Accel[i] * glv.g0) * my_TS;
                            my_v[i] = my_v[i] + my_vm[i];
                        }
                        my_vm[0] = (mpu_Data_value.Accel[0] * glv.g0 - temp_acc_bias[0] * glv.g0) * my_TS;
                        my_vm[1] = (mpu_Data_value.Accel[1] * glv.g0 - temp_acc_bias[1] * glv.g0) * my_TS;
                        my_vm[2] = (mpu_Data_value.Accel[2] * glv.g0 - (temp_acc_bias[2] + 1) * glv.g0) * my_TS;
                        // printf("my_vm: (%f,%f,%f)\n", my_vm[0], my_vm[1], my_vm[2]);
                        printf("my_v: (%f,%f,%f)\n", my_v[0], my_v[1], my_v[2]);
#endif
                        // linearAcc.x = mpu_Data_value.Accel[0] * glv.g0;
                        // linearAcc.y = mpu_Data_value.Accel[1] * glv.g0;
                        // linearAcc.z = mpu_Data_value.Accel[2] * glv.g0;
                        // angles.pitch = (kf.sins.att.i / DEG);
                        // angles.roll = (kf.sins.att.j / DEG);
                        // angles.yaw = (kf.sins.att.k / DEG);
                        // acc_result = get_acc_without_g(linearAcc, gravity, angles);
                        // printf("acceleration without G: %f, %f, %f\n", acc_result.x, acc_result.y, acc_result.z);
                        // printf("Gravity Acceleration: (%f, %f, %f)\n", acc_result.Gx, acc_result.Gy, acc_result.Gz);
                        // printf("Gravity Acceleration 2: (%f, %f, %f)\n", acc_result.Gx / glv.g0, acc_result.Gy / glv.g0, acc_result.Gz / glv.g0);

                        AVPUartOut(kf);
                    }
                }
            }

#ifdef USING_DMP
            data_updated = true;
#endif
#if defined USING_RAW && !defined DOWN_SAMPLING
            timer3_flag = false;
#endif
        }
    }

/* -------------------------------------------------------------------------- */
/*                                  初始校准+捷联惯导                                 */
/* -------------------------------------------------------------------------- */
#elif 1
    float my_vm[3] = {0};
    float temp_gyro_bias[3] = {0};
    float temp_acc_bias[3] = {0};
    double yaw0 = C360CC180(0 * glv.deg); // 北偏东为正
    CVect3 pos0 = LLH(LATITUDE, LONGITUDE, ALTITUDE);
    CVect3 wm = O31;
    CVect3 v00 = O31;
    CVect3 vm = O31;
    CVect3 db = O31;
    CVect3 eb = O31;
    // CSINS sins;
    CSINS sins((CVect3(0, 0, yaw0)), O31, pos0);
    CAlignkf aln(sins, my_TS);
    bool wait_ok = false;
    bool alnOK = false;
    bool aln_init_OK = false;
    bool zero_bias_cal_ok = false;
    int bias_count = 0;
    printf("Initalizing!!\n");
    while (1) {
        if (xSemaphoreTake(xCountingSemaphore_data_update, portMAX_DELAY) == pdTRUE) {
            if (zero_bias_cal_ok == false) {
                if (wait_ok == false) {
                    if (OUT_cnt > WAIT_TIME)
                        wait_ok = true;
                }
                else {
                    for (int i = 0; i < 3; i++) {
                        temp_gyro_bias[i] = temp_gyro_bias[i] + mpu_Data_value.Gyro[i];
                        // temp_acc_bias[i] = temp_acc_bias[i] + mpu_Data_value.Accel[i];
                    }
                    temp_acc_bias[0] = temp_acc_bias[0] + mpu_Data_value.Accel[0];
                    temp_acc_bias[1] = temp_acc_bias[1] + mpu_Data_value.Accel[1];
                    temp_acc_bias[2] = temp_acc_bias[2] + (mpu_Data_value.Accel[2] - 1);
                    bias_count = bias_count + 1;
                    if (OUT_cnt > ZERO_BIAS_CAL_TIME + WAIT_TIME) {
                        zero_bias_cal_ok = true;
                        for (int i = 0; i < 3; i++) {
                            temp_gyro_bias[i] = temp_gyro_bias[i] / bias_count;
                            temp_acc_bias[i] = temp_acc_bias[i] / bias_count;
                            printf("temp_gyro_bias[%d] %f\n", i, temp_gyro_bias[i]);
                            printf("temp_acc_bias[%d] %f \n", i, temp_acc_bias[i]);
                        }
                        // eb = CVect3(temp_gyro_bias[0], temp_gyro_bias[1], temp_gyro_bias[2]) * glv.dps; // 陀螺零偏 deg/s
                        eb = CVect3(temp_gyro_bias[0], temp_gyro_bias[1], temp_gyro_bias[2]); // 陀螺零偏 deg/s

                        // kf.Init(CSINS(a2qua(CVect3(0, 0, yaw0)), O31, pos0));                           // 请正确初始化方位和位置
                        // db = CVect3(temp_acc_bias[0], temp_acc_bias[1], temp_acc_bias[2]) * glv.g0;
                        db = CVect3(temp_acc_bias[0], temp_acc_bias[1], temp_acc_bias[2]);
                        printf("Finish calculating bias  SINS!!\n");
                    }
                }
            }
            else {
                wm = (*(CVect3 *)mpu_Data_value.Gyro - eb) * glv.dps * my_TS;
                vm = (*(CVect3 *)mpu_Data_value.Accel - db) * glv.g0 * my_TS;
                if (alnOK == false) {
                    if (aln_init_OK == false) {
                        aln.Init(sins);
                        aln_init_OK = true;
                    }
                    aln.Update(&wm, &vm, 1, my_TS);
                    if (OUT_cnt > (ALIGN_TIME + ZERO_BIAS_CAL_TIME + WAIT_TIME)) {
                        alnOK = true;
                        v00 = CVect3(0, 0, 0);
                        sins.Init(aln.qnb, v00, pos0, aln.kftk);
                        // sins.db = db / glv.g0;
                        // sins.eb = eb / glv.dps;
                        printf("Finish Aligning!!\n");
                        printf("my_TS: %f\n", my_TS);
                    }
                }
                else {
                    sins.Update(&wm, &vm, 1, my_TS);
                    // AVPUartOut(sins);
#if 0
                    for (int i = 0; i < 3; i++) {
                        // my_vm[i] = (mpu_Data_value.Accel[i] * glv.g0 - temp_acc_bias[i] * glv.g0) * my_TS;
                        // my_vm[i] = (mpu_Data_value.Accel[i] * glv.g0) * my_TS;
                        my_v[i] = my_v[i] + my_vm[i];
                    }
                    my_vm[0] = (mpu_Data_value.Accel[0] * glv.g0 - temp_acc_bias[0] * glv.g0) * my_TS;
                    my_vm[1] = (mpu_Data_value.Accel[1] * glv.g0 - temp_acc_bias[1] * glv.g0) * my_TS;
                    my_vm[2] = (mpu_Data_value.Accel[2] * glv.g0 - (temp_acc_bias[2] + 1) * glv.g0) * my_TS;
                    // printf("my_vm: (%f,%f,%f)\n", my_vm[0], my_vm[1], my_vm[2]);
                    printf("my_v: (%f,%f,%f)\n", my_v[0], my_v[1], my_v[2]);
#endif
                    AVPUartOut(q2att(sins.qnb));
                }
            }
#ifdef USING_DMP
            data_updated = true;
#endif
#if defined USING_RAW && !defined DOWN_SAMPLING
            timer3_flag = false;
#endif
        }
    }
#else
    float my_vm[3] = {0};
    CMahony mahony(10.0);
    int bias_count = 0;
    float temp_gyro_bias[3] = {0};
    float temp_acc_bias[3] = {0};
    bool zero_bias_cal_ok = false;
    CVect3 db = O31;
    CVect3 eb = O31;
    while (1) {
        if (xSemaphoreTake(xCountingSemaphore_data_update, portMAX_DELAY) == pdTRUE) // 得到了信号量
        {
            if (zero_bias_cal_ok == false) {
                for (int i = 0; i < 3; i++) {
                    temp_gyro_bias[i] = temp_gyro_bias[i] + mpu_Data_value.Gyro[i];
                    // temp_acc_bias[i] = temp_acc_bias[i] + mpu_Data_value.Accel[i];
                }
                temp_acc_bias[0] = temp_acc_bias[0] + mpu_Data_value.Accel[0];
                temp_acc_bias[1] = temp_acc_bias[1] + mpu_Data_value.Accel[1];
                temp_acc_bias[2] = temp_acc_bias[2] + (mpu_Data_value.Accel[2] - 1);
                bias_count = bias_count + 1;
                if (OUT_cnt > ZERO_BIAS_CAL_TIME) {
                    zero_bias_cal_ok = true;
                    for (int i = 0; i < 3; i++) {
                        temp_gyro_bias[i] = temp_gyro_bias[i] / bias_count;
                        temp_acc_bias[i] = temp_acc_bias[i] / bias_count;
                        printf("temp_gyro_bias[%d] %f\n", i, temp_gyro_bias[i]);
                        printf("temp_acc_bias[%d] %f \n", i, temp_acc_bias[i]);
                    }
                    eb = CVect3(temp_gyro_bias[0], temp_gyro_bias[1], temp_gyro_bias[2]) * glv.dps; // 陀螺零偏 deg/s
                    // kf.Init(CSINS(a2qua(CVect3(0, 0, yaw0)), O31, pos0));                           // 请正确初始化方位和位置
                    db = CVect3(temp_acc_bias[0], temp_acc_bias[1], temp_acc_bias[2]) * glv.g0;
                    printf("Finish calculating bias!!\n");
                }
            }
            else {
                CVect3 wm = (*(CVect3 *)mpu_Data_value.Gyro * glv.dps - eb) * my_TS;
                CVect3 vm = (*(CVect3 *)mpu_Data_value.Accel * glv.g0 - db) * my_TS;
                mahony.Update(wm, vm, my_TS);
#if 1

                /* -------------------------------------------------------------------------- */
                /*                                    带重力分量                                   */
                /* -------------------------------------------------------------------------- */
                my_vm[0] = (mpu_Data_value.Accel[0] * glv.g0 - temp_acc_bias[0] * glv.g0) * my_TS;
                my_vm[1] = (mpu_Data_value.Accel[1] * glv.g0 - temp_acc_bias[1] * glv.g0) * my_TS;
                my_vm[2] = (mpu_Data_value.Accel[2] * glv.g0 - (temp_acc_bias[2] + 1) * glv.g0) * my_TS;

                /* -------------------------------------------------------------------------- */
                /*                                 旋转矩阵去除重力分量                                 */
                /* -------------------------------------------------------------------------- */
                // vector3D linearAcc = {0, 0, 0};
                // vector3D gravity = {0.0, 0.0, -(float)glv.g0};
                // eulerAngles angles = {0, 0, 0};
                // vector3D_G acc_result;

                // linearAcc.x = mpu_Data_value.Accel[0] * glv.g0;
                // linearAcc.y = mpu_Data_value.Accel[1] * glv.g0;
                // linearAcc.z = mpu_Data_value.Accel[2] * glv.g0;
                // angles.pitch = out_data.Att[0];
                // angles.roll = out_data.Att[1];
                // angles.yaw = out_data.Att[2];
                // acc_result = get_acc_without_g(linearAcc, gravity, angles);
                // my_vm[0] = (acc_result.x) * my_TS;
                // my_vm[1] = (acc_result.y) * my_TS;
                // my_vm[2] = (acc_result.z) * my_TS;
                /* -------------------------------------------------------------------------- */
                /*                               欧拉角去除重力分量（有问题）                               */
                /* -------------------------------------------------------------------------- */
                float temp_acc_without_g[3] = {0};
                float temp_pitch;
                float temp_roll;
                temp_pitch = out_data.Att[0];
                temp_roll = out_data.Att[1];

                // temp_acc_without_g[0] = mpu_Data_value.Accel[1] - sin(temp_roll) * cos(temp_pitch);
                // temp_acc_without_g[1] = mpu_Data_value.Accel[2] - cos(temp_roll) * cos(temp_pitch);
                // temp_acc_without_g[2] = mpu_Data_value.Accel[0] + sin(temp_pitch);

                // my_vm[0] = (temp_acc_without_g[0] * glv.g0 - temp_acc_bias[0] * glv.g0) * my_TS;
                // my_vm[1] = (temp_acc_without_g[1] * glv.g0 - temp_acc_bias[1] * glv.g0) * my_TS;
                // my_vm[2] = (temp_acc_without_g[2] * glv.g0 - temp_acc_bias[2] * glv.g0) * my_TS;
                /* -------------------------------------------------------------------------- */
                /*                                    速度更新                                    */
                /* -------------------------------------------------------------------------- */
                for (int i = 0; i < 3; i++) {
                    my_v[i] = my_v[i] + my_vm[i];
                }

                // printf("my_vm: (%f,%f,%f)\n", my_vm[0], my_vm[1], my_vm[2]);
                printf("my_v: (%f,%f,%f)\n", my_v[0], my_v[1], my_v[2]);
#endif
                AVPUartOut(q2att(mahony.qnb));
            }
#ifdef USING_DMP
            data_updated = true;
#endif
#if defined USING_RAW && !defined DOWN_SAMPLING
            timer3_flag = false;
#endif
        }
    }
#endif

#endif

#endif // PSINS_POS && USING_RAW

#ifdef USING_SFANN_SINS
    xCountingSemaphore_data_update = xSemaphoreCreateCounting(200, 0);
    xTaskCreate(sins_pos_data_update, "sins_pos_data_update", 4096, NULL, 4, NULL);
#endif // USING_SFANN_SINS
}
