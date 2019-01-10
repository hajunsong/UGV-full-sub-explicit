#include "UnmannedGroundVehicle.h"

void UnmannedGroundVehicle::cartesian_velocity_chassis() {
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			if (i == j) {
				chs->T0[i][j] = 1;
				chs->T0[i + 3][j + 3] = 1;
			}
			else {
				chs->T0[i][j] = 0;
				chs->T0[i + 3][j + 3] = 0;
			}
			chs->T0[i + 3][j] = 0;
			chs->T0[i][j + 3] = -chs->r0t[i][j];
		}
	}
	tilde(chs->w0, chs->w0t);
	for (int i = 0; i < 3; i++) {
		chs->dr0c[i] = 0;
		for (int j = 0; j < 3; j++) {
			chs->dr0c[i] += chs->w0t[i][j] * chs->rho0[j];
		}
		chs->dr0c[i] += chs->dr0[i];
	}
}