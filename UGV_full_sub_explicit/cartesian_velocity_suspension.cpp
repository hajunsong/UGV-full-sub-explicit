#include "UnmannedGroundVehicle.h"

void UnmannedGroundVehicle::cartesian_velocity_suspension(suspension *sus) {
	for (int j = 0; j < 3; j++) {
		for (int k = 0; k < 3; k++) {
			sus->T1[j][k] = j == k ? 1 : 0;
			sus->T1[j + 3][k + 3] = j == k ? 1 : 0;
			sus->T1[j + 3][k] = 0;
			sus->T1[j][k + 3] = -sus->r1t[j][k];
		}
	}
	mat6661(sus->T1, sus->Y1h, sus->Y1b);
	for (int j = 0; j < 3; j++) {
		sus->dr1[j] = sus->Y1b[j];
		sus->w1[j] = sus->Y1b[j + 3];
	}
	tilde(sus->w1, sus->w1t);
	for (int j = 0; j < 3; j++) {
		sus->dr1c[j] = 0;
		sus->drwc[j] = 0;
		for (int k = 0; k < 3; k++) {
			sus->dr1c[j] += sus->w1t[j][k] * sus->rho1[k];
			sus->drwc[j] += sus->w1t[j][k] * sus->s12[k];
		}
		sus->dr1c[j] += sus->dr1[j];
		sus->drwc[j] += sus->dr1[j];
	}
}