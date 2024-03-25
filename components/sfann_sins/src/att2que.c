#include "./../inc/att2que.h"
#include "./../inc/MyMatrix.h"

#include <math.h>
/*
 * ***************************************
 *  将欧拉角转为四元数：att2que(att)
 *  input：att=[pitch;roll;yaw]=[theta;gamma;psi];           unit：rad
 *  output：qnb=[q0;q1;q2;q3];                                q0为实部
 * ***************************************
 *
 * Arguments    : const double att[3]
 *                double qnb[4]
 * Return Type  : double*
 */
Matrix att2que(const double att[3])
{
    double c1;
    double c2;
    double c3;
    double s1;
    double s2;
    double s3;
    double qnb_data[4] = { 0 };
    Matrix qnb;
    qnb = Create_Matrix(4, 1);

    s1 = sin(att[0] / 2.0);
    s2 = sin(att[1] / 2.0);
    s3 = sin(att[2] / 2.0);
    c1 = cos(att[0] / 2.0);
    c2 = cos(att[1] / 2.0);
    c3 = cos(att[2] / 2.0);
    
    qnb_data[0] = c3 * c1 * c2 - s3 * s1 * s2;
    qnb_data[1] = c3 * s1 * c2 - s3 * c1 * s2;
    qnb_data[2] = s3 * s1 * c2 - c3 * c1 * s2;
    qnb_data[3] = s3 * c1 * c2 - c3 * s1 * s2;

    SetData_Matrix(qnb, qnb_data);
    return qnb;
}

