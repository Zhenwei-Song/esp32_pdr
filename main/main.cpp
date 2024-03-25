/*
 * @Author: Zhenwei Song zhenwei.song@qq.com
 * @Date: 2024-02-28 18:53:28
 * @LastEditors: Zhenwei Song zhenwei.song@qq.com
 * @LastEditTime: 2024-03-25 15:46:09
 * @FilePath: \esp32_positioning\main\main.cpp
 * @Description: 仅供学习交流使用
 * Copyright (c) 2024 by Zhenwei Song, All Rights Reserved.
 */
/*
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
 * 加速度量程设置：2到16g ,使用4g作为量程(使用SPI时默认量程)
 * 陀螺仪量程设置：250到2000deg/s,使用500deg/s作为量程(使用SPI时默认量程)
 * 读取mpu9250数据可采用dmp（i2c，磁力无法读），直接读寄存器（i2c），直接读寄存器（spi，存在数据读不全，速度慢问题）
 * 姿态解析可采用DMP（无磁力信息）、PSINS中解析，以及简化版捷联惯导sfann_sins(未验证)
 *坐标广研院：23.305836,113.572256，59
 * @Description: 仅供学习交流使用
 * Copyright (c) 2024 by Zhenwei Song, All Rights Reserved.
 */
#include "./main.h"
#include "./all_tasks.h"

#include "./../components/mpu9250/inc/empl_driver.h"
#include "./../components/mpu_timer/inc/positioning_timer.h"

#ifdef USING_DMP
#include "./../components/mpu9250/inc/mpu_dmp_driver.h"
#endif // USING_DMP

#ifdef USING_RAW
#include "./../components/mpu9250/inc/mpu9250_raw.h"
#endif // USING_RAW

#ifdef USING_INS
#include "./../components/ins/inc/data_processing.h"
#include "./../components/ins/inc/ins.h"
#endif // USING_INS

#ifdef USING_PSINS
#include "./../components/mpu9250/inc/mpu_dmp_driver.h"
#include "./../components/psins/inc/mcu_init.h"
#endif // USING_PSINS

#ifdef USING_SFANN_SINS
#include "./../components/mpu9250/inc/mpu_dmp_driver.h"

#endif // USING_SFANN_SINS

#include <stdio.h>
#include <stdlib.h>

extern "C" void app_main(void)
{
#ifdef ONLY_ATT
    xCountingSemaphore_data_update = xSemaphoreCreateCounting(200, 0);
#endif // ONLY_ATT
#ifdef PSINS_POS
    xCountingSemaphore_data_update_psins_pos = xSemaphoreCreateCounting(200, 0);
#endif // PSINS_POS
#ifdef USING_SFANN_SINS
    xCountingSemaphore_data_update_sins_pos = xSemaphoreCreateCounting(200, 0);
#endif // USING_SFANN_SINS

#if defined USING_PSINS && defined USING_DMP
    xCountingSemaphore_timeout2 = xSemaphoreCreateCounting(200, 0);
    mpu_dmp_init();
    i2c_gpio_init();
    ins_init();
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    start_i2c_isr();
    xTaskCreate(gpio_task, "gpio_task", 4096, NULL, 10, NULL);
#endif // USING_PSINS && USING_DMP

#if defined USING_PSINS && defined USING_RAW
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
    xTaskCreate(timer3_check_task, "timer3_check_task", 4096, NULL, 4, NULL);
#endif // USING_PSINS && USING_RAW

#ifdef USING_SFANN_SINS
    uint8_t res = 0;
    mpu_init_i2c();
    res = RAW_MPU9250_Init();
    if (res != 0) {
        printf("raw_mPU9250_init() failed %d\n", res);
    }
    positioning_timer_init();
    esp_timer_start_periodic(positioning_time3_timer, TIME3_TIMER_PERIOD);
    xCountingSemaphore_timeout3 = xSemaphoreCreateCounting(200, 0);
    xTaskCreate(timer3_check_task, "timer3_check_task", 4096, NULL, 4, NULL);
#endif // USING_SFANN_SINS

#ifdef USING_SPI
    xCountingSemaphore_timeout1 = xSemaphoreCreateCounting(200, 0);
    mcu_init();
    xTaskCreate(timer1_check_task, "timer1_check_task", 4096, NULL, 4, NULL);
#endif // USING_SPI

#ifdef ONLY_ATT
    xTaskCreate(data_update, "data_update", 4096, NULL, 4, NULL);
#endif // ONLY_ATT
#ifdef PSINS_POS
    printf("check point1\n");
    xTaskCreate(data_update_psins_pos, "data_update_psins_pos", 16384, NULL, 4, NULL);
#endif // PSINS_POS
#ifdef USING_SFANN_SINS
    printf("check point1\n");
    xTaskCreate(data_update_sins_pos, "data_update_sins_pos", 4096, NULL, 4, NULL);
#endif // USING_SFANN_SINS
}
