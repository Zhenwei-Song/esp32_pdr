#include "./all_tasks.h"

#include <stdio.h>
#include <stdlib.h>


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
#include "./../components/psins/inc/KFApp.h"
#include "./../components/psins/inc/mcu_init.h"
#endif // USING_PSINS

#ifdef USING_SFANN_SINS
#include "./../components/mpu9250/inc/mpu_dmp_driver.h"
#include "./../components/sfann_sins/inc/MyMatrix.h"
#include "./../components/sfann_sins/inc/att2que.h"
#include "./../components/sfann_sins/inc/my_insupdate.h"
#include "./../components/sfann_sins/inc/que2att.h"
#include "./../components/sfann_sins/inc/que2mat.h"
#endif // USING_SFANN_SINS

#ifdef USING_DMP
float prev_angle = 0.0;
static bool data_updated = true;
static s_point new_point;

#endif // USING_DMP

#ifdef USING_RAW
short gyr[3], acc[3], mag[3];
#endif // USING_RAW

#ifdef PSINS_ATT
SemaphoreHandle_t xCountingSemaphore_data_update;
#endif // PSINS_ATT

#ifdef PSINS_POS
static CKFApp kf(my_TS);
SemaphoreHandle_t xCountingSemaphore_data_update_psins_pos;
#endif // PSINS_POS

#ifdef USING_SFANN_SINS
static double wm_data[3] = {0};
static double vm_data[3] = {0};
SemaphoreHandle_t xCountingSemaphore_data_update_sins_pos;
#endif // USING_SFANN

#ifdef USING_DMP
void ins_init(void)
{
    new_point.q[0] = 1;
    new_point.q[1] = 0;
    new_point.q[2] = 0;
    new_point.q[3] = 0;
    printf("ins_init finished\n");
}

/**
 * @description: 通过中断获取DMP中传来的数据
 * @param {void} *arg
 * @return {*}
 */
void gpio_task(void *arg)
{
    uint32_t io_num;
    float angle_increment;
    s_point point_got;
    for (;;) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
#if defined USING_PSINS
            if (data_updated == true) { // 上一个数据处理完
                data_updated = false;
                gyro_data_ready_cb();
                dmp_get_data(&point_got);
                for (int i = 0; i < 3; i++) {
                    new_point.acc[i] = point_got.acc[i];
                    new_point.gyr[i] = point_got.gyr[i];
                    new_point.linear_acc[i] = point_got.linear_acc[i];
                    new_point.acc_fifo[i] = point_got.acc_fifo[i];
                    new_point.gyr_fifo[i] = point_got.gyr_fifo[i];
#ifdef GET_RAW_INFO
                    new_point.acc_raw[i] = point_got.acc_raw[i];
                    new_point.gyr_raw[i] = point_got.gyr_raw[i];
#endif // GET_RAW_INFO
                }
                mpu_AD_value.Gyro[0] = -new_point.gyr_fifo[0]; // fifo读数和直接寄存器读数相差一个负号
                mpu_AD_value.Gyro[1] = -new_point.gyr_fifo[1]; // fifo读数和直接寄存器读数相差一个负号
                mpu_AD_value.Gyro[2] = new_point.gyr_fifo[2];
                // mpu_AD_value.Gyro[0] = 0; // fifo读数和直接寄存器读数相差一个负号
                // mpu_AD_value.Gyro[1] = 0; // fifo读数和直接寄存器读数相差一个负号
                // mpu_AD_value.Gyro[2] = 0;
                for (int i = 0; i < 3; i++) {
                    mpu_AD_value.Accel[i] = new_point.acc_fifo[i];
                    mpu_Data_value.Accel[i] = (double)mpu_AD_value.Accel[i] / (double)A_RANGE_NUM;
                    mpu_Data_value.Gyro[i] = (double)mpu_AD_value.Accel[i] / (double)65.5;
                }
                // printf("new_point.acc:(%.3f,%.3f,%.3f)\n", new_point.acc[0], new_point.acc[1], new_point.acc[2]);
                // printf("new_point.linear_acc:(%.3f,%.3f,%.3f)\n", new_point.linear_acc[0], new_point.linear_acc[1], new_point.linear_acc[2]);
                // printf("new_point.gyr(rad):(%.3f,%.3f,%.3f)\n", new_point.gyr[0], new_point.gyr[1], new_point.gyr[2]);
                xSemaphoreGive(xCountingSemaphore_data_update);
            }
#endif // USING_PSINS

#ifdef USING_INS
            new_point = *get_point(&new_point, 0.2, G);
            printf("point get:(%.3f,%.3f,%.3f)\n\n", new_point.position[0], new_point.position[1], new_point.position[2]);
            dmp_get_data(&point_got);
            for (int i = 0; i < 3; i++) {
                new_point.q[i] = point_got.q[i];
                new_point.acc[i] = point_got.acc[i];
                new_point.gyr[i] = point_got.gyr[i];
                new_point.linear_acc[i] = point_got.linear_acc[i];
            }
            new_point.q[3] = point_got.q[3];
            printf("new_point.speed:(%.3f,%.3f,%.3f)\n", new_point.speed[0], new_point.speed[1], new_point.speed[2]);
            printf("new_point.q:(%.3f,%.3f,%.3f,%.3f)\n", new_point.q[0], new_point.q[1], new_point.q[2], new_point.q[3]);
            printf("new_point.gyr:(%.3f,%.3f,%.3f)\n", new_point.gyr[0], new_point.gyr[1], new_point.gyr[2]);
            // MPU_Get_Magnetometer(&imx, &imy, &imz);
            // printf("MPU_Get_Magnetometer:(%d,%d,%d)\n", imx, imy, imz);
            xSemaphoreGive(xCountingSemaphore_data_update);
#endif // USING_INS
        }
    }
}

#endif // USING_DMP

#ifdef USING_RAW
/**
 * @description: 监听timer3超时，定时器3周期查询数据
 * @param {void} *pvParameters
 * @return {*}
 */
void timer3_check_task(void *pvParameters)
{
    while (1) {
        if (xSemaphoreTake(xCountingSemaphore_timeout3, portMAX_DELAY) == pdTRUE) // 得到了信号量
        {
            RAW_MPU_Get_Gyroscope(&gyr[0], &gyr[1], &gyr[2]);     // 读取角速度原始数据
            RAW_MPU_Get_Accelerometer(&acc[0], &acc[1], &acc[2]); // 读取角加速度原始数据
            RAW_MPU_Get_Magnetometer(&mag[0], &mag[1], &mag[2]);  // 读取磁力计原始数据
            printf("Accx:%d,Accy:%d,Accz:%d\nGyrox:%d,Gyroy:%d,Gyroz:%d\nMagx:%d,Magy:%d,Magz:%d\n\n",
                   acc[0], acc[1], acc[2], gyr[0], gyr[1], gyr[2], mag[0], mag[1], mag[2]); // 源数据串口输出
#if defined PSINS_ATT || defined PSINS_POS
            for (int i = 0; i < 3; i++) {
                mpu_AD_value.Accel[i] = acc[i];
                mpu_AD_value.Gyro[i] = gyr[i];
                mpu_AD_value.Mag[i] = mag[i];

                mpu_Data_value.Accel[i] = (double)mpu_AD_value.Accel[i] / (double)A_RANGE_NUM;
                mpu_Data_value.Gyro[i] = (double)mpu_AD_value.Accel[i] / (double)65.5;
                mpu_Data_value.Mag[i] = mpu_AD_value.Mag[i] * (double)0.25 * ((double)1 + ((double)mag_sensitivity[i] - (double)128) / (double)256);
            }
// printf("A_RANGE_NUM:%d\n", A_RANGE_NUM);
// printf("mag_sensitivity:%d,%d,%d\n", mag_sensitivity[0], mag_sensitivity[1], mag_sensitivity[2]);
#endif // defined PSINS_ATT || defined PSINS_POS

#ifdef USING_SFANN_SINS
            for (int i = 0; i < 3; i++) {
                wm_data[i] = gyr[i];
                vm_data[i] = acc[i];
            }
#endif // USING_SFANN_SINS

#ifdef PSINS_ATT
            xSemaphoreGive(xCountingSemaphore_data_update);
#endif
#ifdef PSINS_POS
            xSemaphoreGive(xCountingSemaphore_data_update_psins_pos);
#endif
#ifdef USING_SFANN_SINS
            xSemaphoreGive(xCountingSemaphore_data_update_sins_pos);
#endif
        }
    }
}
#endif // USING_RAW

#ifdef PSINS_ATT
/**
 * @description: 最终的定位输出
 * @param {void} *pvParameters
 * @return {*}
 */
void data_update(void *pvParameters)
{
    CMahony mahony(10.0);
    // CVect3 eb = CVect3(-4.0, 1.3, 0.0) * glv.dps; // 陀螺零偏 deg/s
    CVect3 eb = CVect3(0, 0, 0) * glv.dps; // 陀螺零偏 deg/s
    CVect3 db = O31;
    double tmp1[3];
    double tmp2[3];
    while (1) {
        if (xSemaphoreTake(xCountingSemaphore_data_update, portMAX_DELAY) == pdTRUE) // 得到了信号量
        {
            CVect3 wm = (*(CVect3 *)mpu_Data_value.Gyro * glv.dps - eb) * my_TS;
            CVect3 vm = (*(CVect3 *)mpu_Data_value.Accel * glv.g0 - db) * my_TS;
            for (int i = 0; i < 3; i++) {
                tmp1[i] = (mpu_Data_value.Gyro[i] * glv.dps - 0) * my_TS;
                tmp2[i] = (mpu_Data_value.Accel[i] * glv.dps - 0) * my_TS;
            }
            // printf("glv.dps:%f\n", glv.dps);
            printf("wm:%f,%f,%f\n", tmp1[0], tmp1[1], tmp1[2]);
            printf("vm:%f,%f,%f\n", tmp2[0], tmp2[1], tmp2[2]);
            printf("mpu_AD_value.Accel:(%d,%d,%d)\n", mpu_AD_value.Accel[0], mpu_AD_value.Accel[1], mpu_AD_value.Accel[2]);
            printf("mpu_AD_value.Gyro:(%d,%d,%d)\n", mpu_AD_value.Gyro[0], mpu_AD_value.Gyro[1], mpu_AD_value.Gyro[2]);
            printf("mpu_Data_value.Accel:(%f,%f,%f)\n", mpu_Data_value.Accel[0], mpu_Data_value.Accel[1], mpu_Data_value.Accel[2]);
            printf("mpu_Data_value.Gyro:(%f,%f,%f)\n", mpu_Data_value.Gyro[0], mpu_Data_value.Gyro[1], mpu_Data_value.Gyro[2]);
            printf("mpu_Data_value.Mag:(%f,%f,%f)\n", mpu_Data_value.Mag[0], mpu_Data_value.Mag[1], mpu_Data_value.Mag[2]);
            mahony.Update(wm, vm, my_TS);
            AVPUartOut(q2att(mahony.qnb));
            data_updated = true;
        }
    }
}
#endif // PSINS_ATT

#ifdef PSINS_POS
void data_update_psins_pos(void *pvParameters)
{
#ifdef DEBUG
    printf("check point4\n");
    // CKFApp kf(my_TS);
    printf("check point5\n");
#endif
    double yaw0 = C360CC180(100.0 * glv.deg); // 北偏东为正
    CVect3 gpspos = LLH(LATITUDE, LONGITUDE, ALTITUDE);
    kf.Init(CSINS(a2qua(CVect3(0, 0, yaw0)), O31, gpspos)); // 请正确初始化方位和位置
    CVect3 eb = CVect3(-4.0, 1.3, 0.0) * glv.dps;           // 陀螺零偏 deg/s
    CVect3 db = O31;
#ifdef DEBUG
    printf("check point9\n");
#endif
    while (1) {
#ifdef DEBUG
        printf("check point22\n");
#endif
        if (xSemaphoreTake(xCountingSemaphore_data_update_psins_pos, portMAX_DELAY) == pdTRUE) {
#ifdef DEBUG
            printf("check point2\n");
#endif
            // kf.SetCalcuBurden(TIM2->CNT, 0);
            kf.SetCalcuBurden(100, 0);
            CVect3 wm = (*(CVect3 *)mpu_Data_value.Gyro * glv.dps - eb) * my_TS;
            CVect3 vm = (*(CVect3 *)mpu_Data_value.Accel * glv.g0 - db) * my_TS;
            kf.Update(&wm, &vm, 1, my_TS, 3);
            AVPUartOut(kf);
            timer3_flag = false;
        }
    }
}
#endif // PSINS_POS

#ifdef USING_SFANN_SINS
void data_update_sins_pos(void *pvParameters)
{
#ifdef DEBUG
    printf("check point4\n");
#endif
    double deg2rad;
    deg2rad = pi / 180;
    Matrix vn, pos, qnb, wm, vm, avptq, avpt;
    vn = Create_Matrix(3, 1);
    pos = Create_Matrix(3, 1);
    qnb = Create_Matrix(4, 1);
    wm = Create_Matrix(3, 1);
    vm = Create_Matrix(3, 1);
    avptq = Create_Matrix(14, 1);
    avpt = Create_Matrix(10, 1);
    double att_data[3] = {0, 0, 90 * deg2rad};
    double vn_data[3] = {0, 0, 0};
    double pos_data[3] = {34 * deg2rad, 108 * deg2rad, 100};
    double qnb_data[4] = {0};
    double ts = 0.2, nts = 0.0;
    // double wm_data[3] = {0};
    // double vm_data[3] = {0};

    qnb = att2que(att_data);
    SetData_Matrix(vn, vn_data);
    SetData_Matrix(pos, pos_data);
#ifdef DEBUG
    printf("check point9\n");
#endif
    while (1) {
        if (xSemaphoreTake(xCountingSemaphore_data_update_sins_pos, portMAX_DELAY) == pdTRUE) {
#ifdef DEBUG
            printf("check point2\n");
#endif
            SetData_Matrix(wm, wm_data);
            SetData_Matrix(vm, vm_data);
            nts = nts + ts;
            avptq = my_insupdate(qnb, vn, pos, wm, vm, nts);
            avpt = Cope_Matrix(avptq, 10, 1);

            vn_data[0] = PickInMat(avptq, 4, 1);
            vn_data[1] = PickInMat(avptq, 5, 1);
            vn_data[2] = PickInMat(avptq, 6, 1);
            pos_data[0] = PickInMat(avptq, 7, 1);
            pos_data[1] = PickInMat(avptq, 8, 1);
            pos_data[2] = PickInMat(avptq, 9, 1);
            qnb_data[0] = PickInMat(avptq, 11, 1);
            qnb_data[1] = PickInMat(avptq, 12, 1);
            qnb_data[2] = PickInMat(avptq, 13, 1);
            qnb_data[3] = PickInMat(avptq, 14, 1);

            SetData_Matrix(vn, vn_data);
            SetData_Matrix(pos, pos_data);
            SetData_Matrix(qnb, qnb_data);
            ShowWrite_Matrix(Trans_Matrix(avptq));
            timer3_flag = false;
        }
    }
}
#endif // USING_SFANN_SINS

#ifdef USING_SPI
/**
 * @description: 监听timer1超时，定时器1周期查询数据
 * @param {void} *pvParameters
 * @return {*}
 */
void timer1_check_task(void *pvParameters)
{
    while (1) {
        if (xSemaphoreTake(xCountingSemaphore_timeout1, portMAX_DELAY) == pdTRUE) // 得到了信号量
        {
            READ_MPU9250_A_T_G();
            // READ_MPU9250_MAG(); // 读MPU9250：IMU和磁
            //  printf("timer1_check_task\n");
            xSemaphoreGive(xCountingSemaphore_data_update);
            timer1_flag = false;
        }
    }
}

#endif // USING_SPI