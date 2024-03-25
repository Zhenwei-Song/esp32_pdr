#include "./../inc/data_processing.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static ps_quaternion updated_attitude_estimation(ps_quaternion q, float *gyr, float dt);

static ps_quaternion rv_to_quaternion(float *rv);

static ps_quaternion quaternion_multiply(ps_quaternion q1, ps_quaternion q2);

static ps_quaternion quaternion_normalization(ps_quaternion q);

static ps_v three_d_rotation(ps_quaternion q, ps_v vi);

/**
 * @description: 角度转换为弧度
 * @param {float} deg
 * @return {*}
 */
float deg_to_rad(float deg)
{
    float rad = 0;
    rad = deg / 180 * PI;
#ifdef DATA_PROCESSING_DEBUG
    printf("deg2rad %f\n", rad);
#endif
    return rad;
}

/**
 * @description: 获取当前点的坐标
 * @param {ps_point} old_point
 * @param {float} dt
 * @param {float} gravity
 * @return {*}
 */
ps_point get_point(ps_point old_point, float dt, float gravity)
{
    ps_point new_point = (ps_point)malloc(sizeof(s_point));
    s_quaternion q_temp;
    s_point last_point;
    s_v v_old;
    q_temp.w = old_point->q[0];
    q_temp.x = old_point->q[1];
    q_temp.y = old_point->q[2];
    q_temp.z = old_point->q[3];
    v_old.x = old_point->acc[0];
    v_old.y = old_point->acc[1];
    v_old.z = old_point->acc[2];
#ifdef DATA_PROCESSING_DEBUG
    printf("old_point.q [%f][%f][%f][%f]\n", old_point->q[0], old_point->q[1], old_point->q[2], old_point->q[3]);
    printf("old_point.gyr [%f][%f][%f]\n", old_point->gyr[0], old_point->gyr[1], old_point->gyr[2]);
    printf("old_point.acc [%f][%f][%f]\n", old_point->acc[0], old_point->acc[1], old_point->acc[2]);
    printf("old_point.speed [%f][%f][%f]\n", old_point->speed[0], old_point->speed[1], old_point->speed[2]);
#endif // DATA_PROCESSING_DEBUG
    // ps_quaternion q_est_temp = updated_attitude_estimation(&q_temp, old_point->gyr, dt);
#ifdef DATA_PROCESSING_DEBUG
    // printf("q_est_temp.q [%f][%f][%f][%f]\n", q_est_temp->w, q_est_temp->x, q_est_temp->y, q_est_temp->z);
#endif // DATA_PROCESSING_DEBUG
    // memcpy(last_point.speed, old_point->speed, sizeof(float) * 3);
    // memcpy(last_point.acc, old_point->acc, sizeof(float) * 3);
    // memcpy(last_point.q, q_est_temp, sizeof(float) * 4);
    memcpy(last_point.q, old_point->q, sizeof(float) * 4);
    // ps_v v_temp = three_d_rotation(q_est_temp, &v_old);
#ifdef DATA_PROCESSING_DEBUG
    // printf("v_temp [%f][%f][%f]\n", v_temp->x, v_temp->y, v_temp->z);
#endif // DATA_PROCESSING_DEBUG

#if 0
    last_point.acc[0] = v_temp->x;
    last_point.acc[1] = v_temp->y;
    last_point.acc[2] = v_temp->z - gravity;
#elif 1
    last_point.acc[0] = old_point->linear_acc[0];
    last_point.acc[1] = old_point->linear_acc[1];
    last_point.acc[2] = old_point->linear_acc[2];
#else
    last_point.acc[0] = 0;
    last_point.acc[1] = 0;
    last_point.acc[2] = 0;
#endif

#ifdef DATA_PROCESSING_DEBUG
    printf("last_point.acc [%f][%f][%f]\n", last_point.acc[0], last_point.acc[1], last_point.acc[2]);
#endif // DATA_PROCESSING_DEBUG
    // free(q_est_temp);
    // free(v_temp);
#ifdef DATA_PROCESSING_DEBUG
    // printf("last_point.speed [%f][%f][%f]\n", last_point.speed[0], last_point.speed[1], last_point.speed[2]);
#endif // DATA_PROCESSING_DEBUG
    for (int i = 0; i < 3; i++) {
        last_point.speed[i] = old_point->speed[i] + dt * last_point.acc[i];
        new_point->speed[i] = last_point.speed[i];
        new_point->position[i] = old_point->position[i] + (old_point->speed[i] + last_point.speed[i]) * dt / 2;
    }
#ifdef DATA_PROCESSING_DEBUG
    printf("last_point.speed [%f][%f][%f]\n", last_point.speed[0], last_point.speed[1], last_point.speed[2]);
#endif // DATA_PROCESSING_DEBUG
    return new_point;
}

/**
 * @description: 更新姿态估计
 * @param {float} *q 当前姿态的四元数表示
 * @param {float *} gyr 陀螺仪测量值
 * @param {float} dt 时间间隔
 * @return {*} 返回更新后的q
 */
static ps_quaternion updated_attitude_estimation(ps_quaternion q, float *gyr, float dt)
{
    ps_quaternion q_est = (ps_quaternion)malloc(sizeof(s_quaternion));
    float rv[3];
    for (int i = 0; i < 3; i++) {
        rv[i] = gyr[i] * dt; // 计算旋转向量
    }
#ifdef DATA_PROCESSING_DEBUG
    // printf("rv [%f][%f][%f]\n", rv[0], rv[1], rv[2]);
#endif // DATA_PROCESSING_DEBUG
    ps_quaternion dq = rv_to_quaternion(rv);
    ps_quaternion q_temp = quaternion_multiply(q, dq);
    q_est = quaternion_normalization(q_temp);
    free(dq);
    free(q_temp);
    return q_est;
}

/**
 * @description: 等效旋转矢量转换为四元数
 * @param {float} *rv 旋转矢量
 * @return {*} 返回四元数
 */
static ps_quaternion rv_to_quaternion(float *rv)
{
    ps_quaternion q_from_rv = (ps_quaternion)malloc(sizeof(s_quaternion));
    float nm2;
    float q0;
    float s;
    float nm;
    nm2 = rv[0] * rv[0] + rv[1] * rv[1] + rv[2] * rv[2]; // 模方
    if (nm2 < 1.0e-8) {
        q0 = 1 - nm2 * (1 / 8 - nm2 / 384);
        s = 1 / 2 - nm2 * (1 / 48 - nm2 / 3840);
    }
    else {
        nm = sqrt(nm2);
        q0 = cos(nm / 2);
        s = sin(nm / 2) / nm;
    }
    q_from_rv->w = q0;
    q_from_rv->x = s * rv[0];
    q_from_rv->y = s * rv[1];
    q_from_rv->z = s * rv[2];
    return q_from_rv;
}

/**
 * @description: 两个四元数相乘
 *               将当前姿态四元数 in 与新的旋转四元数 dq 相乘，得到更新后的姿态四元数
 * @param {ps_quaternion} q1
 * @param {ps_quaternion} q2
 * @return {*}
 */
static ps_quaternion quaternion_multiply(ps_quaternion q1, ps_quaternion q2)
{
    ps_quaternion q_multiply = (ps_quaternion)malloc(sizeof(s_quaternion));
    q_multiply->w = q1->w * q2->w - q1->x * q2->x - q1->y * q2->y - q1->z * q2->z;
    q_multiply->x = q1->w * q2->x + q1->x * q2->w + q1->y * q2->z - q1->z * q2->y;
    q_multiply->y = q1->w * q2->y + q1->y * q2->w + q1->z * q2->x - q1->x * q2->z;
    q_multiply->z = q1->w * q2->z + q1->z * q2->w + q1->x * q2->y - q1->y * q2->x;
    return q_multiply;
}

/**
 * @description: 对更新后的姿态四元数进行归一化
 * @param {ps_quaternion} q
 * @return {*}
 */
static ps_quaternion quaternion_normalization(ps_quaternion q)
{
    ps_quaternion q_normal = (ps_quaternion)malloc(sizeof(s_quaternion));
    float norm2 = q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z;
    float norm = sqrt(norm2);
#if 1
    if (norm2 < 1.0e-6) {
        q_normal->w = 1;
        q_normal->x = 0;
        q_normal->y = 0;
        q_normal->z = 0;
    }
    else {
        q_normal->w = q->w / norm;
        q_normal->x = q->x / norm;
        q_normal->y = q->y / norm;
        q_normal->z = q->z / norm;
        if (q_normal->w < 0) { // 确保其第一个分量非负（这是为了保持惯例，因为四元数表示中有两个相等的姿态对应于同一姿态）
            q_normal->w = -q_normal->w;
            q_normal->x = -q_normal->x;
            q_normal->y = -q_normal->y;
            q_normal->z = -q_normal->z;
        }
    }
#else
    q_normal->w = q->w / norm;
    q_normal->x = q->x / norm;
    q_normal->y = q->y / norm;
    q_normal->z = q->z / norm;
    if (q_normal->w < 0) { // 确保其第一个分量非负（这是为了保持惯例，因为四元数表示中有两个相等的姿态对应于同一姿态）
        q_normal->w = -q_normal->w;
        q_normal->x = -q_normal->x;
        q_normal->y = -q_normal->y;
        q_normal->z = -q_normal->z;
    }
#endif
    return q_normal;
}

/**
 * @description: 向量通过四元数做3D旋转
 * @param {ps_quaternion} q
 * @param {ps_v} vi
 * @return {*}
 */
static ps_v three_d_rotation(ps_quaternion q, ps_v vi)
{
    s_quaternion qo;
    ps_v vo = (ps_v)malloc(sizeof(s_v));
    qo.w = -q->x * vi->x - q->y * vi->y - q->z * vi->z;
    qo.x = q->w * vi->x + q->y * vi->z - q->z * vi->y;
    qo.y = q->w * vi->y + q->z * vi->x - q->x * vi->z;
    qo.z = q->w * vi->z + q->x * vi->y - q->y * vi->x;
#ifdef DATA_PROCESSING_DEBUG
    // printf("vi [%f][%f][%f]\n", vi->x, vi->y, vi->z);
    // printf("q [%f][%f][%f][%f]\n", q->w, q->x, q->y, q->z);
    // printf("qo.q [%f][%f][%f][%f]\n", qo.w, qo.x, qo.y, qo.z);
#endif // DATA_PROCESSING_DEBUG
    // vo = vi;
    vo->x = -qo.w * q->x + qo.x * q->w - qo.y * q->z + qo.z * q->y;
    vo->y = -qo.w * q->y + qo.y * q->w - qo.z * q->x + qo.x * q->z;
    vo->z = -qo.w * q->z + qo.z * q->w - qo.x * q->y + qo.y * q->x;
#ifdef DATA_PROCESSING_DEBUG
    // printf("vo [%f][%f][%f]\n", vo->x, vo->y, vo->z);
#endif // DATA_PROCESSING_DEBUG
    return vo;
}