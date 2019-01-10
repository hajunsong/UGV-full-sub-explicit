#include "UnmannedGroundVehicle.h"

void UnmannedGroundVehicle::read_LR(suspension *sus) {
	sus->q1 = LR_in[0];
	sus->dq1 = LR_in[1];

	sus->rho1p[0] = LR_in[2]; sus->rho1p[1] = LR_in[3]; sus->rho1p[2] = LR_in[4];

	sus->C11[0][0] = LR_in[5]; sus->C11[0][1] = LR_in[6]; sus->C11[0][2] = LR_in[7];
	sus->C11[1][0] = LR_in[8]; sus->C11[1][1] = LR_in[9]; sus->C11[1][2] = LR_in[10];
	sus->C11[2][0] = LR_in[11]; sus->C11[2][1] = LR_in[12]; sus->C11[2][2] = LR_in[13];

	sus->m1 = LR_in[14];

	sus->J1p[0][0] = LR_in[15]; sus->J1p[0][1] = LR_in[16]; sus->J1p[0][2] = LR_in[17];
	sus->J1p[1][0] = LR_in[18]; sus->J1p[1][1] = LR_in[19]; sus->J1p[1][2] = LR_in[20];
	sus->J1p[2][0] = LR_in[21]; sus->J1p[2][1] = LR_in[22]; sus->J1p[2][2] = LR_in[23];

	sus->s01p[0] = LR_in[24]; sus->s01p[1] = LR_in[25]; sus->s01p[2] = LR_in[26];

	sus->s12p[0] = LR_in[27]; sus->s12p[1] = LR_in[28]; sus->s12p[2] = LR_in[29];

	sus->C01[0][0] = LR_in[30]; sus->C01[0][1] = LR_in[31]; sus->C01[0][2] = LR_in[32];
	sus->C01[1][0] = LR_in[33]; sus->C01[1][1] = LR_in[34]; sus->C01[1][2] = LR_in[35];
	sus->C01[2][0] = LR_in[36]; sus->C01[2][1] = LR_in[37]; sus->C01[2][2] = LR_in[38];

	sus->C12[0][0] = LR_in[39]; sus->C12[0][1] = LR_in[40]; sus->C12[0][2] = LR_in[41];
	sus->C12[1][0] = LR_in[42]; sus->C12[1][1] = LR_in[43]; sus->C12[1][2] = LR_in[44];
	sus->C12[2][0] = LR_in[45]; sus->C12[2][1] = LR_in[46]; sus->C12[2][2] = LR_in[47];

	sus->s0sp[0] = LR_in[48]; sus->s0sp[1] = LR_in[49]; sus->s0sp[2] = LR_in[50];

	sus->s1sp[0] = LR_in[51]; sus->s1sp[1] = LR_in[52]; sus->s1sp[2] = LR_in[53];

	sus->theta_wh = 0; sus->w_wh = 0; sus->dw_wh = 0;

	sus->rw[0] = chs->A0[0][0] * sus->s01p[0] + chs->A0[0][1] * sus->s01p[1] + chs->A0[0][2] * sus->s01p[2] + chs->r0[0] + (chs->A0[0][0] * sus->C01[0][0] + chs->A0[0][1] * sus->C01[1][0] + chs->A0[0][2] * sus->C01[2][0]) * sus->s12p[0] + (chs->A0[0][0] * sus->C01[0][1] + chs->A0[0][1] * sus->C01[1][1] + chs->A0[0][2] * sus->C01[2][1]) * sus->s12p[1] + (chs->A0[0][0] * sus->C01[0][2] + chs->A0[0][1] * sus->C01[1][2] + chs->A0[0][2] * sus->C01[2][2]) * sus->s12p[2];
	sus->rw[1] = chs->A0[1][0] * sus->s01p[0] + chs->A0[1][1] * sus->s01p[1] + chs->A0[1][2] * sus->s01p[2] + chs->r0[1] + (chs->A0[1][0] * sus->C01[0][0] + chs->A0[1][1] * sus->C01[1][0] + chs->A0[1][2] * sus->C01[2][0]) * sus->s12p[0] + (chs->A0[1][0] * sus->C01[0][1] + chs->A0[1][1] * sus->C01[1][1] + chs->A0[1][2] * sus->C01[2][1]) * sus->s12p[1] + (chs->A0[1][0] * sus->C01[0][2] + chs->A0[1][1] * sus->C01[1][2] + chs->A0[1][2] * sus->C01[2][2]) * sus->s12p[2];
	sus->rw[2] = chs->A0[2][0] * sus->s01p[0] + chs->A0[2][1] * sus->s01p[1] + chs->A0[2][2] * sus->s01p[2] + chs->r0[2] + (chs->A0[2][0] * sus->C01[0][0] + chs->A0[2][1] * sus->C01[1][0] + chs->A0[2][2] * sus->C01[2][0]) * sus->s12p[0] + (chs->A0[2][0] * sus->C01[0][1] + chs->A0[2][1] * sus->C01[1][1] + chs->A0[2][2] * sus->C01[2][1]) * sus->s12p[1] + (chs->A0[2][0] * sus->C01[0][2] + chs->A0[2][1] * sus->C01[1][2] + chs->A0[2][2] * sus->C01[2][2]) * sus->s12p[2];
}