#include "UnmannedGroundVehicle.h"

void UnmannedGroundVehicle::mass_force_state_suspension(suspension *sus) {
	double A1_C11[3][3], A1_C11_J1p[3][3];
	mat3333(sus->A1, sus->C11, A1_C11);
	mat3333(A1_C11, sus->J1p, A1_C11_J1p);
	mat3333T(A1_C11_J1p, A1_C11, sus->J1c);

	tilde(sus->r1c, sus->r1ct);
	tilde(sus->dr1c, sus->dr1ct);

	for (int j = 0; j < 3; j++) {
		for (int k = 0; k < 3; k++) {
			sus->M1h[j][k] = j == k ? sus->m1 : 0;
			sus->M1h[j][k + 3] = -sus->m1*sus->r1ct[j][k];
			sus->M1h[j + 3][k] = sus->m1*sus->r1ct[j][k];
			sus->M1h[j + 3][k + 3] = 0;
			for (int m = 0; m < 3; m++) {
				sus->M1h[j + 3][k + 3] += sus->r1ct[j][m] * sus->r1ct[m][k];
			}
			sus->M1h[j + 3][k + 3] *= -sus->m1;
			sus->M1h[j + 3][k + 3] += sus->J1c[j][k];
		}
	}

	TSDA(sus);
	// Tire input parameter - wheel È¸Àü Ãà vector
	double temp[3][3] = { 0, };
	mat3333(sus->A1, sus->C12, temp);
	double u_vec[3] = { -temp[0][2], -temp[1][2], -temp[2][2] };
	// Tire force & moment calculate
	tire->PNU_Tire_force(sus->rw, sus->drwc, u_vec, sus->w_wh, sus->id);
	// Get result of tire analysis
	tire->PNU_get_data(sus->F_tire, sus->M_tire, &sus->slip, &sus->angle, &sus->road_h, &sus->pen, &sus->R_d, &sus->Fx, &sus->Fy, &sus->Fz, &sus->My);
	sus->kt = 310000;

	sus->R_z = sus->rw[2];
	sus->R_u = tire->PNU_get_unloaded_radius();
	sus->point_h = sus->R_z - sus->R_u;
	sus->pen = sus->road_h - sus->point_h;
	sus->pen = sus->pen >= 0 ? sus->pen : 0;

	memcpy(sus->r_road, sus->rw, sizeof(double) * 2);
	sus->r_road[2] = sus->rw[2] - sus->road_h;
	for (int j = 0; j < 3; j++) {
		sus->r_p2a[j] = -sus->r_road[j] + sus->r1c[j];
	}
	tilde(sus->F_tire, sus->tF_tire);

	sus->F1c[0] = sus->F_tire[0];
	sus->F1c[1] = sus->F_tire[1];
	sus->F1c[2] = sus->m1*g + sus->F_tire[2];
	sus->T1c[0] = sus->M_tire[0];
	sus->T1c[1] = sus->M_tire[1];
	sus->T1c[2] = sus->M_tire[2];

	sus->Q1h_g[0] = sus->m1 * sus->dr1ct[0][0] * sus->w1[0] + sus->m1 * sus->dr1ct[0][1] * sus->w1[1] + sus->m1 * sus->dr1ct[0][2] * sus->w1[2] + sus->F1c[0];
	sus->Q1h_g[1] = sus->m1 * sus->dr1ct[1][0] * sus->w1[0] + sus->m1 * sus->dr1ct[1][1] * sus->w1[1] + sus->m1 * sus->dr1ct[1][2] * sus->w1[2] + sus->F1c[1];
	sus->Q1h_g[2] = sus->m1 * sus->dr1ct[2][0] * sus->w1[0] + sus->m1 * sus->dr1ct[2][1] * sus->w1[1] + sus->m1 * sus->dr1ct[2][2] * sus->w1[2] + sus->F1c[2];
	sus->Q1h_g[3] = sus->r1ct[0][0] * sus->F1c[0] + sus->r1ct[0][1] * sus->F1c[1] + sus->r1ct[0][2] * sus->F1c[2] + sus->T1c[0] + (sus->m1 * sus->r1ct[0][0] * sus->dr1ct[0][0] + sus->m1 * sus->r1ct[0][1] * sus->dr1ct[1][0] + sus->m1 * sus->r1ct[0][2] * sus->dr1ct[2][0]) * sus->w1[0] + (sus->m1 * sus->r1ct[0][0] * sus->dr1ct[0][1] + sus->m1 * sus->r1ct[0][1] * sus->dr1ct[1][1] + sus->m1 * sus->r1ct[0][2] * sus->dr1ct[2][1]) * sus->w1[1] + (sus->m1 * sus->r1ct[0][0] * sus->dr1ct[0][2] + sus->m1 * sus->r1ct[0][1] * sus->dr1ct[1][2] + sus->m1 * sus->r1ct[0][2] * sus->dr1ct[2][2]) * sus->w1[2] - (sus->w1t[0][0] * sus->J1c[0][0] + sus->w1t[0][1] * sus->J1c[1][0] + sus->w1t[0][2] * sus->J1c[2][0]) * sus->w1[0] - (sus->w1t[0][0] * sus->J1c[0][1] + sus->w1t[0][1] * sus->J1c[1][1] + sus->w1t[0][2] * sus->J1c[2][1]) * sus->w1[1] - (sus->w1t[0][0] * sus->J1c[0][2] + sus->w1t[0][1] * sus->J1c[1][2] + sus->w1t[0][2] * sus->J1c[2][2]) * sus->w1[2];
	sus->Q1h_g[4] = sus->r1ct[1][0] * sus->F1c[0] + sus->r1ct[1][1] * sus->F1c[1] + sus->r1ct[1][2] * sus->F1c[2] + sus->T1c[1] + (sus->m1 * sus->r1ct[1][0] * sus->dr1ct[0][0] + sus->m1 * sus->r1ct[1][1] * sus->dr1ct[1][0] + sus->m1 * sus->r1ct[1][2] * sus->dr1ct[2][0]) * sus->w1[0] + (sus->m1 * sus->r1ct[1][0] * sus->dr1ct[0][1] + sus->m1 * sus->r1ct[1][1] * sus->dr1ct[1][1] + sus->m1 * sus->r1ct[1][2] * sus->dr1ct[2][1]) * sus->w1[1] + (sus->m1 * sus->r1ct[1][0] * sus->dr1ct[0][2] + sus->m1 * sus->r1ct[1][1] * sus->dr1ct[1][2] + sus->m1 * sus->r1ct[1][2] * sus->dr1ct[2][2]) * sus->w1[2] - (sus->w1t[1][0] * sus->J1c[0][0] + sus->w1t[1][1] * sus->J1c[1][0] + sus->w1t[1][2] * sus->J1c[2][0]) * sus->w1[0] - (sus->w1t[1][0] * sus->J1c[0][1] + sus->w1t[1][1] * sus->J1c[1][1] + sus->w1t[1][2] * sus->J1c[2][1]) * sus->w1[1] - (sus->w1t[1][0] * sus->J1c[0][2] + sus->w1t[1][1] * sus->J1c[1][2] + sus->w1t[1][2] * sus->J1c[2][2]) * sus->w1[2];
	sus->Q1h_g[5] = sus->r1ct[2][0] * sus->F1c[0] + sus->r1ct[2][1] * sus->F1c[1] + sus->r1ct[2][2] * sus->F1c[2] + sus->T1c[2] + (sus->m1 * sus->r1ct[2][0] * sus->dr1ct[0][0] + sus->m1 * sus->r1ct[2][1] * sus->dr1ct[1][0] + sus->m1 * sus->r1ct[2][2] * sus->dr1ct[2][0]) * sus->w1[0] + (sus->m1 * sus->r1ct[2][0] * sus->dr1ct[0][1] + sus->m1 * sus->r1ct[2][1] * sus->dr1ct[1][1] + sus->m1 * sus->r1ct[2][2] * sus->dr1ct[2][1]) * sus->w1[1] + (sus->m1 * sus->r1ct[2][0] * sus->dr1ct[0][2] + sus->m1 * sus->r1ct[2][1] * sus->dr1ct[1][2] + sus->m1 * sus->r1ct[2][2] * sus->dr1ct[2][2]) * sus->w1[2] - (sus->w1t[2][0] * sus->J1c[0][0] + sus->w1t[2][1] * sus->J1c[1][0] + sus->w1t[2][2] * sus->J1c[2][0]) * sus->w1[0] - (sus->w1t[2][0] * sus->J1c[0][1] + sus->w1t[2][1] * sus->J1c[1][1] + sus->w1t[2][2] * sus->J1c[2][1]) * sus->w1[1] - (sus->w1t[2][0] * sus->J1c[0][2] + sus->w1t[2][1] * sus->J1c[1][2] + sus->w1t[2][2] * sus->J1c[2][2]) * sus->w1[2];

}