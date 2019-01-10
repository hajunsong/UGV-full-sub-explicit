#include "UnmannedGroundVehicle.h"

void UnmannedGroundVehicle::velocity_coupling(suspension *sus) {
	tilde(chs->dr0, chs->dr0t);
	tilde(sus->dr1, sus->dr1t);
	mat3331(chs->w0t, sus->H1, sus->dH1);

	for (int j = 0; j < 3; j++) {
		sus->D1_temp[j] = 0;
		for (int k = 0; k < 3; k++) {
			sus->D1_temp[j] += sus->dr1t[j][k] * sus->H1[k] + sus->r1t[j][k] * sus->dH1[k];
		}
		sus->D1_temp[j + 3] = sus->dH1[j];
		sus->D1[j] = sus->D1_temp[j] * sus->dq1;
		sus->D1[j + 3] = sus->D1_temp[j + 3] * sus->dq1;
	}
}