/*
 * @Author: Zhenwei Song zhenwei.song@qq.com
 * @Date: 2024-02-28 18:53:28
 * @LastEditors: Zhenwei Song zhenwei.song@qq.com
 * @LastEditTime: 2024-02-29 15:02:34
 * @FilePath: \esp32_pdr\mpu9250\main\main.c
 * 驱动mpu9250，串口输出欧拉角，可用上位机进行串口连接查看图像
 * 利用官方dmp库输出欧拉角
 * 使用I2C连接
 * 引脚使用：INT 23 SCL 22  SDA 21 供电3.3v
 * 采样率设置：最快为200Hz(5ms采集一次)     文件mpu_dmp_driver.h   #define DEFAULT_MPU_HZ
 * 加速度量程设置：2到16g   文件inv_mpu.c    mpu_set_accel_fsr(16)      注意：同步修改mpu_dmp_driver.c中换算部分
 * @Description: 仅供学习交流使用
 * Copyright (c) 2024 by Zhenwei Song, All Rights Reserved.
 */

#include <stdio.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
// #include "freertos/task.h"
// #include "freertos/queue.h"
#include "driver/gpio.h"
// #include "oled_driver.h"
#include "mpu_dmp_driver.h"

// #define GPIO_MPU_INTR 4
#define GPIO_MPU_INTR 23

// 存储需要刷新的表情
// expression_t expression = {
//     .forehead_expr       = forehead_none,
//     .eyes_expr           = eyes_none,
//     .nose_expr           = nose_none,
//     .mouth_expr          = mouth_none,
//     .chin_expr           = chin_none,
//     .forehead_refresh_ena= 1,
//     .eyes_refresh_ena    = 1,
//     .nose_refresh_ena    = 1,
//     .mouth_refresh_ena   = 1,
//     .chin_refresh_ena    = 1,
//     .frame_delay_ms = 10
// };

extern float pitch, roll, yaw;
static QueueHandle_t gpio_evt_queue = NULL; // 用于接收 GPIO 中断的 queue

// 初始化所有需要使用的GPIO
void gpio_init(void);
void gpio_task(void *arg);
void gpio_intr_handle(void *arg);

void app_main(void)
{
    // oled_init();
    mpu_dmp_init();
    gpio_init();
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(gpio_task, "gpio_task", 2048, NULL, 10, NULL);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(GPIO_MPU_INTR, gpio_intr_handle, (void *)GPIO_MPU_INTR);
}

void gpio_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_MPU_INTR),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    gpio_config(&io_conf);
}

void gpio_task(void *arg)
{
    uint32_t io_num;
    for (;;) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            gyro_data_ready_cb();
            dmp_get_data();
            // printf("pitch:%f,roll:%f,yaw:%f\n", pitch, roll, yaw);
            //  expression.frame_delay_ms = (uint16_t)(esp_random() % 10);
            //  if (abs(pitch) > 30)
            //  {
            //      expression.forehead_expr = arc;
            //      expression.eyes_expr = wink;
            //  }
            //  else
            //  {
            //      expression.forehead_expr = forehead_none;
            //      expression.eyes_expr = eyes_none;
            //  }
            //  oled_refresh_expression(expression);
        }
    }
}

void gpio_intr_handle(void *arg)
{
    uint32_t gpio_num = (uint32_t)arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}