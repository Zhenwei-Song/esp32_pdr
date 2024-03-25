/*
 * ***************************************
 *  四元数转换为姿态角：que2att(qnb)
 *  input：qnb=[q0;q1;q2;q3];             q0为实部
 *  output：att=[pitch;roll;yaw];           unit：rad
 * ***************************************
 *
 * Arguments    : const double qnb[4]
 *                double att[3]
 * Return Type  : void
 */
#include "./../inc/que2att.h"

#include <math.h>

Matrix que2att(Matrix qnb)
{
    Matrix att;
    att = Create_Matrix(3, 1);
    double att_tmp;
    double att_data[3] = { 0 };
    att_tmp = 2.0 * (PickInMat(qnb, 3, 1) * PickInMat(qnb, 4, 1) + PickInMat(qnb, 1, 1) * PickInMat(qnb, 2, 1));
    att_data[0] = asin(att_tmp);
    if (fabs(att_tmp) <= 0.999999) 
    {
        att_data[1] = -atan2(2.0 * (PickInMat(qnb, 2, 1) * PickInMat(qnb, 4, 1) - PickInMat(qnb, 1, 1) * PickInMat(qnb, 3, 1)),
                      PickInMat(qnb, 1, 1) * PickInMat(qnb, 1, 1) - PickInMat(qnb, 2, 1) * PickInMat(qnb, 2, 1) - PickInMat(qnb, 3, 1) * PickInMat(qnb, 3, 1) + PickInMat(qnb, 4, 1) * PickInMat(qnb, 4, 1));
        /*  atan2(y,x);           四象限反正切 */
        att_data[2] = -atan2(2.0 * (PickInMat(qnb, 2, 1) * PickInMat(qnb, 3, 1) - PickInMat(qnb, 1, 1) * PickInMat(qnb, 4, 1)),
                      PickInMat(qnb, 1, 1) * PickInMat(qnb, 1, 1) - PickInMat(qnb, 2, 1) * PickInMat(qnb, 2, 1) + PickInMat(qnb, 3, 1) * PickInMat(qnb, 3, 1) - PickInMat(qnb, 4, 1) * PickInMat(qnb, 4, 1));
    }
    else {
        att_data[1] = atan2(2.0 * (PickInMat(qnb, 2, 1) * PickInMat(qnb, 4, 1) + PickInMat(qnb, 1, 1) * PickInMat(qnb, 3, 1)),
                      PickInMat(qnb, 1, 1) * PickInMat(qnb, 1, 1) + PickInMat(qnb, 2, 1) * PickInMat(qnb, 2, 1) - PickInMat(qnb, 3, 1) * PickInMat(qnb, 3, 1) - PickInMat(qnb, 4, 1) * PickInMat(qnb, 4, 1));
        att_data[2] = 0.0;
    }
    SetData_Matrix(att, att_data);
    return att;
}

