#include "UnmannedGroundVehicle.h"

void UnmannedGroundVehicle::TSDA(suspension *sus) {
	sus->L_free = 0.8;

	mat3331(chs->A0, sus->s0sp, sus->s0s);
	mat3331(sus->A1, sus->s1sp, sus->s1s);

	for (int j = 0; j < 3; j++) {
		sus->d01[j] = sus->r1[j] + sus->s1s[j] - chs->r0[j] - sus->s0s[j];
	}
	sus->poly_factor[0] = 4.631e+09;
	sus->poly_factor[1] = -1.492e-06;
	sus->poly_factor[2] = -2.063e+08;
	sus->poly_factor[3] = 6.644e-08;
	sus->poly_factor[4] = 3.638e+06;
	sus->poly_factor[5] = -6.597e-10;
	sus->poly_factor[6] = 2279;
	sus->poly_factor[7] = -3000;

	sus->L_spring = sqrt(pow(sus->d01[0], 2) + pow(sus->d01[1], 2) + pow(sus->d01[2], 2));
	sus->defo = sus->L_spring - sus->L_free;

	sus->T_spring = 0;
	for (int j = 0; j < 8; j++) {
		sus->T_spring += sus->poly_factor[j] * pow(sus->defo, 7 - j);
	}

	for (int j = 0; j < 3; j++) {
		sus->dL_spring_temp[j] = 0;
		for (int k = 0; k < 3; k++) {
			sus->dL_spring_temp[j] += sus->w1t[j][k] * sus->s1s[k] - chs->w0t[j][k] * sus->s0s[k];
		}
		sus->dL_spring_temp[j] += sus->dr1[j] - chs->dr0[j];
	}
	sus->dL_spring = (sus->d01[0] * sus->dL_spring_temp[0] + sus->d01[1] * sus->dL_spring_temp[1] + sus->d01[2] * sus->dL_spring_temp[2]) / sus->L_spring;

	sus->C = 31500;
	sus->T_damper = sus->dL_spring*sus->C;

	sus->f = sus->T_spring + sus->T_damper;

	tilde(sus->s0s, sus->s0st);
	tilde(sus->s1s, sus->s1st);

	sus->f_L = sus->f / sus->L_spring;

	for (int j = 0; j < 3; j++) {
		sus->Q0h_TSDA_temp[j + 3] = 0;
		sus->Q1h_TSDA_temp[j + 3] = 0;
		for (int k = 0; k < 3; k++) {
			sus->Q0h_TSDA_temp[j + 3] += (chs->r0t[j][k] + sus->s0st[j][k])*sus->d01[k];
			sus->Q1h_TSDA_temp[j + 3] += (sus->r1t[j][k] + sus->s1st[j][k])*sus->d01[k];
		}
		sus->Q0h_TSDA_temp[j] = sus->d01[j];
		sus->Q1h_TSDA_temp[j] = sus->d01[j];
	}
	for (int j = 0; j < 6; j++) {
		sus->Q0h_TSDA[j] = sus->f_L*sus->Q0h_TSDA_temp[j];
		sus->Q1h_TSDA[j] = -sus->f_L*sus->Q1h_TSDA_temp[j];
	}
}