#include "./../inc/KFApp.h"

/***************************  class CKFApp  *********************************/
CKFApp::CKFApp(double ts):CSINSGNSS(19, 6, ts)
{
//state: 0-2 phi; 3-5 dvn; 6-8 dpos; 9-11 eb; 12-14 db; 15-17 lever; 18 dt
//meas:  0-2 dvn; 3-5 dpos
printf("check point11\n");
SetCalcuBurden(100, -1);
printf("check point12\n");
}

void CKFApp::Init(const CSINS &sins0, int grade)
{
	CSINSGNSS::Init(sins0);
	Pmax.Set2(fPHI(600,600),  fXXX(500),  fdPOS(1e6),  fDPH3(5000),  fMG3(10), fXXX(10),  0.1);
	Pmin.Set2(fPHI(0.1,1.0),  fXXX(0.001),  fdPOS(0.1),  fDPH3(0.1),  fUG3(10), fXXX(0.01),  0.0001);
	Pk.SetDiag2(fPHI(60,600),  fXXX(1.0),  fdPOS(10.0),  fDPH3(100),  fMG3(3.0), fXXX(1.0),  0.01);
	Qt.Set2(fDPSH3(0.1),  fUGPSHZ3(1.0),  fOOO,  fOO6,	fOOO, 0.0);
	Rt.Set2(fXXZ(0.5,1.0),   fdLLH(10.0,30.0));
	Rmax = Rt*100;  Rmin = Rt*0.01;  Rb = 0.6;
	FBTau.Set(fXX9(0.1),  fXX6(1.0),  fINF3, INF);
}

void AVPUartOut(const CKFApp &kf)
{
    printf("kf.sins.att: %f,%f,%f\n", kf.sins.att.i, kf.sins.att.j, kf.sins.att.k);
    AVPUartOut(kf.sins.att, kf.sins.vn, kf.sins.pos);
}

void AVPUartOut(const CVect3 &att, const CVect3 &vn, const CVect3 &pos)
{
	out_data.Att[0] = att.i/DEG; out_data.Att[1] = att.j/DEG; out_data.Att[2] = CC180C360(att.k)/DEG;
	out_data.Vn[0] = vn.i; out_data.Vn[1] = vn.j; out_data.Vn[2] = vn.k;
	int deg;
	deg = (int)(pos.j/DEG);
	out_data.Pos[0] = deg;  out_data.Pos[1] = pos.j/DEG-deg;
	deg = (int)(pos.i/DEG);
	out_data.Pos[2] = deg;  out_data.Pos[3] = pos.i/DEG-deg;
	out_data.Pos[4] = pos.k;
    printf("out att: %f,%f,%f\n", out_data.Att[0], out_data.Att[1],out_data.Att[2]);
    printf("out Vn: %f,%f,%f\n", out_data.Vn[0], out_data.Vn[1], out_data.Vn[2]);
    printf("out Pos: %f,%f,%f,%f,%f\n\n", out_data.Pos[0], out_data.Pos[1], out_data.Pos[2], out_data.Pos[3], out_data.Pos[4]);
}
