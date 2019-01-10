#include "UnmannedGroundVehicle.h"

void UnmannedGroundVehicle::velocity_state_suspension(suspension *sus) {
	tilde(sus->r1, sus->r1t);
	for (int j = 0; j < 3; j++) {
		sus->B1[j] = 0;
		for (int k = 0; k < 3; k++) {
			sus->B1[j] += sus->r1t[j][k] * sus->H1[k];
		}
		sus->B1[j + 3] = sus->H1[j];
	}
	for (int j = 0; j < 6; j++) {
		sus->Y1h[j] = chs->Y0h[j] + sus->B1[j] * sus->dq1;
	}
}