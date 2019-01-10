#include "UnmannedGroundVehicle.h"

void UnmannedGroundVehicle::position_chassis() {
	mat3331(chs->A0, chs->rho0p, chs->rho0);
	for (int i = 0; i < 3; i++) {
		chs->r0c[i] = chs->r0[i] + chs->rho0[i];
	}
}