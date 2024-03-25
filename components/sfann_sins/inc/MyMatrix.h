#ifndef MYMATRIX_H
#define MYMATRIX_H
#ifdef __cplusplus
extern "C" {
#endif
typedef struct MNode* PtrToMNode;
struct MNode
{
	int row;
	int column;
	double** data;
};
typedef PtrToMNode Matrix;

//　创建矩阵
Matrix Create_Matrix(int row, int column);
// 释放申请的矩阵空间
void Free_Matrix(Matrix mat);
// 初始化矩阵(将所有元素初始化为０)
void Init_Matrix(Matrix mat);
// 给矩阵每个元素赋值
void SetData_Matrix(Matrix mat, double data[]);
//　打印矩阵
void Show_Matrix(Matrix mat, char* s);
// 科学计数法打印输出
void Show_Matrix_E(Matrix mat, char* s);
// 打印输出并写入文件
void ShowWrite_Matrix(Matrix mat);
//　矩阵加减法
Matrix AddorSub_Matrix(Matrix mat_1, Matrix mat_2, int flag);
//　矩阵转置
Matrix Trans_Matrix(Matrix mat);
// 矩阵乘法
Matrix Mult_Matrix(Matrix mat_1, Matrix mat_2);
// 矩阵点乘
Matrix DotMul_Matrix(Matrix mat_1, Matrix mat_2);
// 矩阵数乘
Matrix MatrixMulNum(const Matrix mat, double num);
// 四元数乘法
Matrix QMul(Matrix q1, Matrix q2);
// 向量取模
double Norm_Matrix(const Matrix mat);
// 四元数归一化
Matrix Norm_Matrix_Q(const Matrix q);
//矩阵复制
Matrix Cope_Matrix(Matrix mat, int row, int column);
//　取出矩阵某行某列的元素
double PickInMat(Matrix mat, int r, int c);
// 向量叉乘
Matrix Cross(Matrix a, Matrix b);

////　创建单位矩阵
//Matrix eye(int n);
////　顺序高斯消去法
//Matrix Gauss_shunxu(Matrix A, Matrix b);
////　列主元高斯消去法
//Matrix Gauss_lie(Matrix A, Matrix b);
//Matrix LUInverse_Matrix(Matrix mat);
//
//// 矩阵求逆，利用初等行变换求逆
//Matrix EleTransInv_Matrix(Matrix mat);
//// 矩阵第n行与第m行互换
//void Swap_row(Matrix mat, int n, int m);
//
//
//double Det_Matrix(Matrix mat);
////　伴随矩阵(还没写)
//Matrix Adj_Matrix(Matrix mat);
////　对一个矩阵进行复制
//Matrix Copy_Matrix(Matrix mat);
#ifdef __cplusplus
}
#endif
#endif 

