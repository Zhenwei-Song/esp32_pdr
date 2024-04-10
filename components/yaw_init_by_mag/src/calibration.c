#include "./../inc/calibration.h"

/*
** 磁力计采用离线校准，采集球面数据后导入matlab
** 使用matlab提供的magcal函数得到矫正参数(b = 零点偏移) (A = 矫正矩阵)
** 使用方式参考官方手册：https://ww2.mathworks.cn/help/nav/ref/magcal.html?s_tid=doc_ta
** https://ww2.mathworks.cn/help/nav/ug/magnetometer-calibration.html?searchHighlight=magcal&s_tid=srchtitle_magcal_3
** 将离线校准所得结果更新到下边的 A b变量内即可。
*/

const CVect3 b(-0.042351227, -0.10260590, 0.12779139);
const CMat3 A(0.99960005, -0.00053128414, -0.0010546446,
              -0.00053128414, 0.99418974, 0.0042561144,
              -0.0010546446, 0.0042561144, 1.0062668);

// const CVect3 b(-0.064778641, -0.070484757, 0.13823533);
// const CMat3 A(1.0003734,	-0.0045067165,	-0.0015763268,
//							-0.0045067165,	0.98665541,	0.0026261732,
//							-0.0015763268,	0.0026261732,	1.0131767);

// void mag_correct(const float *in, float *Out)
//{
//	float tmp[3] = {0.0};
//	tmp[0] = in[0] - b[0];
//	tmp[1] = in[1] - b[1];
//	tmp[2] = in[2] - b[2];
//
//	Out[0] = A[0] * tmp[0] + A[3] * tmp[1] + A[6] * tmp[2];
//	Out[1] = A[1] * tmp[0] + A[4] * tmp[1] + A[7] * tmp[2];
//	Out[2] = A[2] * tmp[0] + A[5] * tmp[1] + A[8] * tmp[2];
// }