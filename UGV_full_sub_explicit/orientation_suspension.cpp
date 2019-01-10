#include "UnmannedGroundVehicle.h"

void UnmannedGroundVehicle::orientation_suspension(suspension *sus) {
	double A0_C01[3][3];
	double u_vec[3] = { 0,0,1 };

	sus->A01pp[0][0] = cos(sus->q1);
	sus->A01pp[0][1] = -sin(sus->q1);
	sus->A01pp[0][2] = 0;
	sus->A01pp[1][0] = sin(sus->q1);
	sus->A01pp[1][1] = cos(sus->q1);
	sus->A01pp[1][2] = 0;
	sus->A01pp[2][0] = 0;
	sus->A01pp[2][1] = 0;
	sus->A01pp[2][2] = 1;

	mat3333(chs->A0, sus->C01, A0_C01);
	mat3333(A0_C01, sus->A01pp, sus->A1);

	mat3331(A0_C01, u_vec, sus->H1);
}