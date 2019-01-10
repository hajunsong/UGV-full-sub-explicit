#include "UnmannedGroundVehicle.h"

void UnmannedGroundVehicle::acceleration_state_chassis() {
	const int n = subsystems;
	double fac[subsystems][subsystems] = { 0, };
	int indx[subsystems] = { 0, };

	memcpy(M, chs->M0h, sizeof(double) * 6 * 6);

	memcpy(Q, chs->Q0h_g, sizeof(double) * 6);
	memcpy(chs->L0, chs->Q0h_g, sizeof(double) * 6);

	for (int i = 0; i < 6; i++) {
		for (int j = 0; j < 6; j++) {
			Q[j] += sus[i].Q0h_TSDA[j];
			chs->L0[j] += sus[i].Q0h_TSDA[j];
		}
	}

	for (int i = 0; i < 6; i++) {
		for (int j = 0; j < 6; j++) {
			for (int k = 0; k < 6; k++) {
				M[j][k] += +sus[i].Mhc[j][k];
			}
		}
	}

	for (int i = 0; i < 6; i++) {
		for (int j = 0; j < 6; j++) {
			Q[j] += sus[i].Qhc[j];
		}
	}

	ludcmp6(M, n, indx, 0.0, fac);
	lubksb6(fac, n, indx, Q, chs->dY0h);
}