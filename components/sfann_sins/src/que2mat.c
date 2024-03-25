#include "./../inc/que2mat.h"

Matrix que2mat(Matrix q)
{
	Matrix mat;
	mat = Create_Matrix(3, 3);
	double q11, q12, q13, q14, q22, q23, q24, q33, q34, q44;
	q11 = PickInMat(q, 1, 1) * PickInMat(q, 1, 1); //qnb[0] * qnb[0];
	q12 = PickInMat(q, 1, 1) * PickInMat(q, 2, 1); //qnb[0] * qnb[1];
	q13 = PickInMat(q, 1, 1) * PickInMat(q, 3, 1); //qnb[0] * qnb[2];
	q14 = PickInMat(q, 1, 1) * PickInMat(q, 4, 1); //qnb[0] * qnb[3];
	q22 = PickInMat(q, 2, 1) * PickInMat(q, 2, 1); //qnb[1] * qnb[1];
	q23 = PickInMat(q, 2, 1) * PickInMat(q, 3, 1); //qnb[1] * qnb[2];
	q24 = PickInMat(q, 2, 1) * PickInMat(q, 4, 1); //qnb[1] * qnb[3];
	q33 = PickInMat(q, 3, 1) * PickInMat(q, 3, 1); //qnb[2] * qnb[2];
	q34 = PickInMat(q, 3, 1) * PickInMat(q, 4, 1); //qnb[2] * qnb[3];
	q44 = PickInMat(q, 4, 1) * PickInMat(q, 4, 1); //qnb[3] * qnb[3];
	double mat_date[9] = { q11 + q22 - q33 - q44,    2 * (q23 - q14),         2 * (q24 + q13),
                           2 * (q23 + q14),          q11 - q22 + q33 - q44,   2 * (q34 - q12),
                           2 * (q24 - q13),          2 * (q34 + q12),         q11 - q22 - q33 + q44 };
	SetData_Matrix(mat, mat_date); 
	return mat;
}
