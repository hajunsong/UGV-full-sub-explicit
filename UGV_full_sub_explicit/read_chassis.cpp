#include "UnmannedGroundVehicle.h"

void UnmannedGroundVehicle::read_chassis() {

	//chs->r0[0] = CH_in[0];	chs->r0[1] = CH_in[1];	chs->r0[2] = CH_in[2];
	//chs->p0[0] = CH_in[3];	chs->p0[1] = CH_in[4];	chs->p0[2] = CH_in[5];	chs->p0[3] = CH_in[6];

	chs->r0[0] = input->LocalPath[0][0];//WP[0][0][0]; // 초기 글로벌 X;
	chs->r0[1] = input->LocalPath[0][1];//WP[0][1][0]; // 초기 글로벌 Y;
	chs->r0[2] = 0;

	chs->p0[0] = sqrt(2 * cos(heading) + 2) / 2;
	chs->p0[1] = 0;
	chs->p0[2] = 0;
	chs->p0[3] = (sin(heading) * 2) / (4 * chs->p0[0]);

	chs->A0[0][0] = 2 * (chs->p0[0] * chs->p0[0] + chs->p0[1] * chs->p0[1] - 0.5);
	chs->A0[0][1] = 2 * (chs->p0[1] * chs->p0[2] - chs->p0[0] * chs->p0[3]);
	chs->A0[0][2] = 2 * (chs->p0[1] * chs->p0[3] + chs->p0[0] * chs->p0[2]);
	chs->A0[1][0] = 2 * (chs->p0[1] * chs->p0[2] + chs->p0[0] * chs->p0[3]);
	chs->A0[1][1] = 2 * (chs->p0[0] * chs->p0[0] + chs->p0[2] * chs->p0[2] - 0.5);
	chs->A0[1][2] = 2 * (chs->p0[2] * chs->p0[3] - chs->p0[0] * chs->p0[1]);
	chs->A0[2][0] = 2 * (chs->p0[1] * chs->p0[3] - chs->p0[0] * chs->p0[2]);
	chs->A0[2][1] = 2 * (chs->p0[2] * chs->p0[3] + chs->p0[0] * chs->p0[1]);
	chs->A0[2][2] = 2 * (chs->p0[0] * chs->p0[0] + chs->p0[3] * chs->p0[3] - 0.5);

 	read_RF(&sus[RF]);
	read_RM(&sus[RM]);
	read_RR(&sus[RR]);
	read_LF(&sus[LF]);
	read_LM(&sus[LM]);
	read_LR(&sus[LR]);

	// 차량 초기 자세에서 각 타이어 별 침투량 계산
	for (int i = 0; i < 6; i++) {
		sus[i].road_h = 0;
		sus[i].pen = sus[i].road_h - (sus->rw[2] - tire->PNU_get_unloaded_radius());
	}

	// 차체의 높이를 설정하기 위해 각 타이어 침투량 중 최대값을 찾는 routine
	double max_pen = 0;
	for (int i = 0; i < 6; i++) {
		if (max_pen < sus[i].pen) max_pen = sus[i].pen;
	}
	chs->r0[2] = max_pen;

	chs->dr0[0] = CH_in[7]; chs->dr0[1] = CH_in[8]; chs->dr0[2] = CH_in[9];
	chs->w0[0] = CH_in[10]; chs->w0[1] = CH_in[11]; chs->w0[2] = CH_in[12];

	chs->rho0p[0] = CH_in[13]; chs->rho0p[1] = CH_in[14]; chs->rho0p[2] = CH_in[15];

	chs->C00[0][0] = CH_in[16]; chs->C00[0][1] = CH_in[17]; chs->C00[0][2] = CH_in[18];
	chs->C00[1][0] = CH_in[19]; chs->C00[1][1] = CH_in[20]; chs->C00[1][2] = CH_in[21];
	chs->C00[2][0] = CH_in[22]; chs->C00[2][1] = CH_in[23]; chs->C00[2][2] = CH_in[24];

	chs->m0 = CH_in[25];

	chs->J0p[0][0] = CH_in[26]; chs->J0p[0][1] = CH_in[27]; chs->J0p[0][2] = CH_in[28];
	chs->J0p[1][0] = CH_in[29]; chs->J0p[1][1] = CH_in[30]; chs->J0p[1][2] = CH_in[31];
	chs->J0p[2][0] = CH_in[32]; chs->J0p[2][1] = CH_in[33]; chs->J0p[2][2] = CH_in[34];
}