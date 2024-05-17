#include "./../inc/my_insupdate.h"
#include "./../inc/que2att.h"
#include "./../inc/att2que.h"
#include "./../inc/rv2que.h"
#include "./../inc/que2mat.h"

#include "./../../../main/main.h"

#include <string.h>
#include <math.h>
#include <stdlib.h>

#define g0  9.780325333434361
#define f  1/298.257223563
#define R_e  6.378136998405e6
#define w_ie  7.2921151467e-5
#define pi 3.1415926535897932384626433832795

Matrix my_insupdate(Matrix qnb, Matrix vn1, Matrix pos, Matrix wm, Matrix vm, double nts)
{
	double ts = 0.01;
	double sl, sl2, sl4, gLh;
	sl = sin(PickInMat(pos, 1, 1));
	sl2 = sl * sl;  sl4 = sl2 * sl2;
	gLh = g0 * (1 + 5.27094e-3 * sl2 + 2.32718e-5 * sl4) - 3.086e-6 * PickInMat(pos, 3, 1);
	double gn_data[3] = { 0, 0, -gLh };
	Matrix gn, M_pv, wnie, wnen, wnen0, wnin, cnb, att;
	gn = Create_Matrix(3, 1);
	wnie = Create_Matrix(3, 1);
	att = Create_Matrix(3, 1);
	cnb = Create_Matrix(3, 3);
	double att_data[3] = { 0 };
	double cnb_data[3] = { 0 };
	SetData_Matrix(gn, gn_data);
	//Show_Matrix_E(gn, "gn = ");//

	double e, R_N, R_M, R_Mh, R_Nh;
	e = sqrt(2.0 * f - pow(f, 2));
	R_N = R_e / sqrt(1.0 - pow(e, 2) * sl2);
	R_M = R_N * (1.0 - pow(e, 2)) / (1.0 - pow(e, 2) * sl2);
	R_Mh = R_M + PickInMat(pos, 3, 1);
	R_Nh = R_N + PickInMat(pos, 3, 1);
	M_pv = Create_Matrix(3, 3);
	double M_pv_data[9] = { 0, 1.0 / R_Mh, 0, 1.0 / (R_Nh * cos(PickInMat(pos, 1, 1))), 0, 0, 0, 0, 1 };
	SetData_Matrix(M_pv, M_pv_data);
	//Show_Matrix_E(M_pv, "M_pv = ");//
	double wnie_data[3] = { 0, w_ie * cos(PickInMat(pos, 1, 1)), w_ie * sin(PickInMat(pos, 1, 1)) };
	SetData_Matrix(wnie, wnie_data);
	wnen0 = Create_Matrix(3, 1);
	double wnen_data0[3] = {-1.0 / (R_M + PickInMat(pos, 3, 1)), 1.0 / (R_N + PickInMat(pos, 3, 1)), tan(PickInMat(pos, 1, 1)) / (R_N + PickInMat(pos, 3, 1))};
	SetData_Matrix(wnen0, wnen_data0);
	wnen = DotMul_Matrix(vn1, wnen0);
	wnin = AddorSub_Matrix(wnie, wnen, 0);
	//Show_Matrix_E(wnin, "wnin = ");//

	Matrix vsf, vn2, q_bb, qbb_2row, avptq, vo;
	vsf = Create_Matrix(3, 1);
	vn2 = Create_Matrix(3, 1);
	q_bb = Create_Matrix(4, 1);
	qbb_2row = Create_Matrix(3, 1);
	avptq = Create_Matrix(14, 1);
	vo = Create_Matrix(3, 1);
	vo = AddorSub_Matrix(vm, MatrixMulNum(Cross(wm, vm), 0.5), 0);
	//Show_Matrix_E(vo, "vo = ");//
	//Show_Matrix_E(qnb, "qnb = ");//
	cnb = que2mat(qnb);
	//Show_Matrix_E(cnb, "cnb = ");//
	vsf = Mult_Matrix(cnb,vo);
	//Show_Matrix_E(vsf, "vsf = ");//
	vn2 = AddorSub_Matrix(AddorSub_Matrix(vn1, vsf, 0), MatrixMulNum(gn, ts), 0);
	//Show_Matrix_E(vn1, "vn1 = ");//
	//Show_Matrix_E(AddorSub_Matrix(vn1, vsf, 0), "AddorSub_Matrix(vn1, vsf, 0) = ");//
	//Show_Matrix_E(MatrixMulNum(gn, nts), "MatrixMulNum(gn, nts) = ");//
	//Show_Matrix_E(vn2, "vn2 = ");//
	vn1 = MatrixMulNum(AddorSub_Matrix(vn1, vn2, 0), 0.5);
	//Show_Matrix_E(vn1, "vn1 = ");//
	//Show_Matrix_E(pos, "pos = ");//
	pos = AddorSub_Matrix(pos, MatrixMulNum(Mult_Matrix(M_pv, vn1), ts), 0);
	//Show_Matrix_E(pos, "pos = ");//
	//Show_Matrix_E(wm, "wm = ");//
	
	double ss = sin(Norm_Matrix(wm) / 2.0);
	//printf("%.5e", ss);//
	qbb_2row = MatrixMulNum(MatrixMulNum(wm, 1.0 / Norm_Matrix(wm)), ss);
	double q_bb_data[4] = { cos(Norm_Matrix(wm) / 2.0), PickInMat(qbb_2row, 1, 1), PickInMat(qbb_2row, 2, 1), PickInMat(qbb_2row, 3, 1) };
	SetData_Matrix(q_bb, q_bb_data);
	//Show_Matrix(q_bb, "qbb = ");//
	qnb = QMul(rv2que(MatrixMulNum(wnin, -ts)), QMul(qnb, q_bb));
	//Show_Matrix_E(qnb, "qnb_befor = ");//
	qnb = Norm_Matrix_Q(qnb);
	//Show_Matrix_E(qnb, "qnb_after = ");//
	att = que2att(qnb);
	//Show_Matrix(att, "att = ");//
	att_data[0] = PickInMat(att, 1, 1);
	att_data[1] = PickInMat(att, 2, 1);
	att_data[2] = fmod(PickInMat(att, 3, 1), 2 * pi);
	SetData_Matrix(att, att_data);

    out_data.Att[0] = (float)att_data[0];
    out_data.Att[1] = (float)att_data[1];
    out_data.Att[2] = (float)att_data[2];

    //Show_Matrix(att, "att = ");//

	//æÿ’Û∆¥Ω”
	double cvpta_data[14] = { PickInMat(att, 1, 1), PickInMat(att, 2, 1), PickInMat(att, 3, 1),
		                    PickInMat(vn2, 1, 1), PickInMat(vn2, 2, 1), PickInMat(vn2, 3, 1),
		                    PickInMat(pos, 1, 1), PickInMat(pos, 2, 1), PickInMat(pos, 3, 1),
		                    nts,
	                        PickInMat(qnb, 1, 1), PickInMat(qnb, 2, 1), PickInMat(qnb, 3, 1), PickInMat(qnb, 4, 1) };

	SetData_Matrix(avptq, cvpta_data);
	return avptq;
}