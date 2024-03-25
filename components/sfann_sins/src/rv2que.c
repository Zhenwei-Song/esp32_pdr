#include "./../inc/rv2que.h"

#include <math.h>

/* 旋转矢量转化为四元数 */
  /*  */
  /*  Prototype: q = rv2q(rv) */
  /*  Input: rv - rotation vector */
  /*  Output: q - corresponding transformation quaternion, such that */
  /*              q = [ cos(|rv|/2); sin(|rv|/2)/|rv|*rv ] */
  /*   */
  /*  See also  q2rv, rv2m, m2rv, a2qua, rotv, qupdt. */
  /*  Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved. */
  /*  Northwestern Polytechnical University, Xi An, P.R.China */
  /*  05/02/2009, 22/05/2014 */

Matrix rv2que(Matrix _rv)
{
	Matrix q;
	q = Create_Matrix(4, 1);
	//q = zeros(4, 1);
	double q_data[4] = { 0 };
	double n2, s, n, n_2;
	n2 = PickInMat(_rv, 1, 1) * PickInMat(_rv, 1, 1) + PickInMat(_rv, 2, 1) * PickInMat(_rv, 2, 1) + PickInMat(_rv, 3, 1) * PickInMat(_rv, 3, 1);

	if (n2 < 1.0e-8)
	{
		q_data[0] = 1 - n2 * (1.0 / 8 - n2 / 384);
		s = 1.0 / 2 - n2 * (1.0 / 48 - n2 / 3840);
	}else{
		n = sqrt(n2); n_2 = n / 2;
		q_data[0] = cos(n_2); s = sin(n_2) / n;
	}
		q_data[1] = s * PickInMat(_rv, 1, 1); 
		q_data[2] = s * PickInMat(_rv, 2, 1); 
		q_data[3] = s * PickInMat(_rv, 3, 1);
		SetData_Matrix(q, q_data);
	return q;
}
