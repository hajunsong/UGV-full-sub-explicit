#include "UnmannedGroundVehicle.h"

void UnmannedGroundVehicle::TIRE(suspension *sus) {
	double road_h = -0.58;

	sus->road_h = road_h;

	sus->kt = 310000;

	for (int j = 0; j < 3; j++) {
		sus->rw[j] = 0;
		for (int k = 0; k < 3; k++) {
			sus->rw[j] += sus->A1[j][k] * sus->s12p[k];
		}
		sus->rw[j] += sus->r1[j];
	}
	sus->R_z = sus->rw[2];
	sus->R_u = 0.548;
	sus->point_h = sus->R_z - sus->R_u;
	sus->pen = sus->road_h - sus->point_h;

	if (sus->pen > 0) {
		sus->Fz = sus->kt*sus->pen;
	}
	else {
		sus->Fz = 0;
	}

	sus->F_tire[0] = 0;
	sus->F_tire[1] = 0;
	sus->F_tire[2] = sus->Fz;

	memcpy(sus->r_road, sus->rw, sizeof(double) * 2);
	sus->r_road[2] = sus->rw[2] - sus->road_h;
	for (int j = 0; j < 3; j++) {
		sus->r_p2a[j] = -sus->r_road[j] + sus->r1c[j];
	}
	tilde(sus->F_tire, sus->tF_tire);
	mat3331(sus->tF_tire, sus->r_p2a, sus->M_tire);
}