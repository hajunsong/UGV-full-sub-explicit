#include "UnmannedGroundVehicle.h"

void UnmannedGroundVehicle::position_suspension(suspension *sus) {
	mat3331(chs->A0, sus->s01p, sus->s01);
	mat3331(sus->A1, sus->s12p, sus->s12);
	mat3331(sus->A1, sus->rho1p, sus->rho1);
	for (int j = 0; j < 3; j++) {
		sus->r1[j] = chs->r0[j] + sus->s01[j];
		sus->rw[j] = sus->r1[j] + sus->s12[j];
		sus->r1c[j] = sus->r1[j] + sus->rho1[j];
	}
}