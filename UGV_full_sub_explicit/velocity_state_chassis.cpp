#include "UnmannedGroundVehicle.h"

void UnmannedGroundVehicle::velocity_state_chassis() {
	tilde(chs->r0, chs->r0t);
	for (int i = 0; i < 3; i++) {
		chs->Y0h[i] = 0;
		for (int j = 0; j < 3; j++) {
			chs->Y0h[i] += chs->r0t[i][j] * chs->w0[j];
		}
		chs->Y0h[i] += chs->dr0[i];
		chs->Y0h[i + 3] = chs->w0[i];
	}
}