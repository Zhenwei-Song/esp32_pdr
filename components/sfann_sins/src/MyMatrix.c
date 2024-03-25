#include "./../inc/MyMatrix.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// 初始化矩阵
void Init_Matrix(Matrix mat)
{
	int i, j;
	for (i = 0; i < mat->row; i++) {
		for (j = 0; j < mat->column; j++) {
			mat->data[i][j] = 0;
		}
	}
}

// 释放申请的矩阵空间
void Free_Matrix(Matrix mat)
{
	for (int i = 0; i < mat->row; i++)
		free(mat->data[i]); // 释放行指针
	free(mat->data); // 释放头指针
	free(mat); // 释放结构体指针
}

Matrix Create_Matrix(int row, int col)
{
	Matrix mat;
	mat = (Matrix)malloc(sizeof(struct MNode)); //　分配结构体指针
	if (row <= 0 || col <= 0) {
		printf("ERROR, in creat_Matrix the row or col <= 0\n");
		exit(1);
	}
	if (row > 0 && col > 0) {
		mat->row = row;
		mat->column = col;
		mat->data = (double**)malloc(row * sizeof(double*));// 分配头指针
		if (mat->data == NULL) {
			printf("ERROR, in creat_Matrix the mat->data == NULL\n");
			exit(1);
		}
		int i;
		for (i = 0; i < row; i++) {
			*(mat->data + i) = (double*)malloc(col * sizeof(double)); //　分配每行的指针
			if (mat->data[i] == NULL) {
				printf("ERROR, in create_Matrix the mat->data[i] == NULL\n");
				exit(1);
			}
		}
		Init_Matrix(mat);
	}
	return mat;
}

void Show_Matrix(Matrix mat, char* s)
{
	int i, j;
	printf("%s\n", s);
	for (i = 0; i < mat->row; i++) {
		for (j = 0; j < mat->column; j++)
			printf("%.13f\t", mat->data[i][j]);
		printf("\n");
	}
	printf("\n");
}

void Show_Matrix_E(Matrix mat, char* s)
{
	int i, j;
	printf("%s\n", s);
	for (i = 0; i < mat->row; i++) {
		for (j = 0; j < mat->column; j++)
			printf("%.13e\t", mat->data[i][j]);//%1.5e
		printf("\n");
	}
	printf("\n");
}

void ShowWrite_Matrix(Matrix mat)
{
	int i, j;
	for (i = 0; i < mat->row; i++) {
		for (j = 0; j < mat->column; j++)
		{
			printf("%f		\t", mat->data[i][j]);
		}
		printf("\n");
	}
	printf("\n");
}

void SetData_Matrix(Matrix mat, double data[])
{
	int i, j;
	for (i = 0; i < mat->row; i++) {
		for (j = 0; j < mat->column; j++) {
			mat->data[i][j] = data[i * mat->column + j];
		}
	}
}

double PickInMat(Matrix mat, int r, int c)
{
	double rst;
	rst = mat->data[r - 1][c - 1];
	return rst;
}

Matrix AddorSub_Matrix(Matrix mat_1, Matrix mat_2, int flag)
{
	Matrix rst_mat;
	if (mat_1->column != mat_2->column) {
		printf("ERROR in AddorSub, column !=\n");
		exit(1);
	}
	if (mat_1->row != mat_2->row) {
		printf("ERROR in AddorSub, row !=\n");
		exit(1);
	}
	int i, j;
	rst_mat = Create_Matrix(mat_1->row, mat_1->column);
	for (i = 0; i < mat_1->row; i++) {
		for (j = 0; j < mat_1->column; j++)
			rst_mat->data[i][j] = mat_1->data[i][j] + pow(-1, flag) * mat_2->data[i][j];
	}
	return rst_mat;
}

Matrix DotMul_Matrix(Matrix mat_1, Matrix mat_2)
{
	Matrix rst_mat;
	if (mat_1->column != mat_2->column) {
		printf("ERROR in AddorSub, column !=\n");
		exit(1);
	}
	if (mat_1->row != mat_2->row) {
		printf("ERROR in AddorSub, row !=\n");
		exit(1);
	}
	int i, j;
	rst_mat = Create_Matrix(mat_1->row, mat_1->column);
	for (i = 0; i < mat_1->row; i++) {
		for (j = 0; j < mat_1->column; j++)
			rst_mat->data[i][j] = mat_1->data[i][j] * mat_2->data[i][j];
	}
	return rst_mat;
}
//3 * 3
Matrix Cross(Matrix a, Matrix b)
{
	Matrix C;
	C = Create_Matrix(a->row, a->column);
	double ax, ay, az;
	double bx, by, bz;
	double c[3] = { 0 };
	ax = PickInMat(a, 1, a->column);
	ay = PickInMat(a, 2, a->column);
	az = PickInMat(a, 3, a->column);
	bx = PickInMat(b, 1, b->column);
	by = PickInMat(b, 2, b->column);
	bz = PickInMat(b, 3, b->column);
	c[0] = ay * bz - az * by;
	c[1] = az * bx - ax * bz;
	c[2] = ax * by - ay * bx;
	SetData_Matrix(C, c);
	return C;
}

Matrix Mult_Matrix(Matrix mat_1, Matrix mat_2)
{
	Matrix rst_mat;
	int i, j, m;
	if (mat_1->column != mat_2->row) {
		printf("ERROR in Mult_Matrix, column != row\n");
		exit(1);
	}
	else {
		rst_mat = Create_Matrix(mat_1->row, mat_2->column);
		for (i = 0; i < mat_1->row; i++) {
			for (j = 0; j < mat_2->column; j++) {
				for (m = 0; m < mat_1->column; m++)
					rst_mat->data[i][j] += mat_1->data[i][m] * mat_2->data[m][j];
			}
		}
	}
	return rst_mat;
}

Matrix MatrixMulNum(const Matrix mat, double num) 
{
	Matrix rst_mat;
	int i, j;
	rst_mat = Create_Matrix(mat->row, mat->column);
	for (i = 0; i < mat->row; i++) {
		for (j = 0; j < mat->column; j++)
			rst_mat->data[i][j] = mat->data[i][j] * num;
	}
	return rst_mat;
}
//vector magnitude
double Norm_Matrix(const Matrix mat)
{
	Matrix rst_mat;
	double ret = 0;
	rst_mat = Create_Matrix(mat->row, mat->column);
	for (int i = 0; i < mat->column; i++)
	{
		rst_mat->data[i][0] = mat->data[i][0] * mat->data[i][0];
		ret += rst_mat->data[i][0];
	}
	return sqrt(ret);
}

Matrix Norm_Matrix_Q(const Matrix q)
{
	Matrix rst_mat;
	double rst_data[4] = { 1, 0, 0, 0 };
	rst_mat = Create_Matrix(q->row, q->column);
	double nm = 0;
	for (int i = 0; i < q->row; i++)
	{
		rst_mat->data[i][0] = q->data[i][0] * q->data[i][0];
		nm += rst_mat->data[i][0];
	}
	if (nm < 1e-6){
		SetData_Matrix(rst_mat, rst_data);
	}
	else {
		rst_mat = MatrixMulNum(q, 1.0 / sqrt(nm));
	}  
	return rst_mat;
}

Matrix Trans_Matrix(Matrix mat)
{
	Matrix mat_;
	int i, j;
	mat_ = Create_Matrix(mat->column, mat->row);
	for (i = 0; i < mat->column; i++) {
		for (j = 0; j < mat->row; j++)
			mat_->data[i][j] = mat->data[j][i];
	}
	return mat_;
}

Matrix Cope_Matrix(Matrix mat, int row, int column)
{
	Matrix mat_;
	int i, j;
	mat_ = Create_Matrix(row, column);
	for (i = 0; i < row; i++) {
		for (j = 0; j < column; j++)
			mat_->data[i][j] = mat->data[i][j];
	}
	return mat_;
}

Matrix zeros(int row, int column)
{
	Matrix mat;
	int i, j;
	mat = Create_Matrix(row, column);
	for (i = 0; i < row; i++) {
		for (j = 0; j < column; j++)
			mat->data[i][j] = 0;
	}
	return mat;
}

Matrix QMul(Matrix q1, Matrix q2)
{
	Matrix q;
	q = Create_Matrix(4, 1);
	double q_date[4] = { 
		PickInMat(q1, 1, 1) * PickInMat(q2, 1, 1) - PickInMat(q1, 2, 1) * PickInMat(q2, 2, 1) - PickInMat(q1, 3, 1) * PickInMat(q2, 3, 1) - PickInMat(q1, 4, 1) * PickInMat(q2, 4, 1), 
		PickInMat(q1, 1, 1) * PickInMat(q2, 2, 1) + PickInMat(q1, 2, 1) * PickInMat(q2, 1, 1) + PickInMat(q1, 3, 1) * PickInMat(q2, 4, 1) - PickInMat(q1, 4, 1) * PickInMat(q2, 3, 1),
		PickInMat(q1, 1, 1) * PickInMat(q2, 3, 1) + PickInMat(q1, 3, 1) * PickInMat(q2, 1, 1) + PickInMat(q1, 4, 1) * PickInMat(q2, 2, 1) - PickInMat(q1, 2, 1) * PickInMat(q2, 4, 1),
		PickInMat(q1, 1, 1) * PickInMat(q2, 4, 1) + PickInMat(q1, 4, 1) * PickInMat(q2, 1, 1) + PickInMat(q1, 2, 1) * PickInMat(q2, 3, 1) - PickInMat(q1, 3, 1) * PickInMat(q2, 2, 1),
	};
	SetData_Matrix(q, q_date);
	return q;
}
