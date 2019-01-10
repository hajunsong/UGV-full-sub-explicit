#include "UnmannedGroundVehicle.h"

void UnmannedGroundVehicle::cartesian_acceleration_suspension(suspension *sus) {
	memset(sus->dT1, 0, sizeof(double) * 6 * 6);
	for (int j = 0; j < 3; j++) {
		for (int k = 0; k < 3; k++) {
			sus->dT1[j][k + 3] = -sus->dr1t[j][k];
		}
	}
	for (int j = 0; j < 6; j++) {
		sus->dY1b[j] = 0;
		for (int k = 0; k < 6; k++) {
			sus->dY1b[j] += sus->dT1[j][k] * sus->Y1h[k] + sus->T1[j][k] * sus->dY1h[k];
		}
	}
	memcpy(sus->ddr1, sus->dY1b, sizeof(double) * 3);
	memcpy(sus->dw1, sus->dY1b + 3, sizeof(double) * 3);

	tilde(sus->dw1, sus->dw1t);

	sus->ddr1c[0] = sus->dw1t[0][0] * sus->rho1[0] + sus->dw1t[0][1] * sus->rho1[1] + sus->dw1t[0][2] * sus->rho1[2] + sus->ddr1[0] + (pow(sus->w1t[0][0], 0.2e1) + sus->w1t[0][1] * sus->w1t[1][0] + sus->w1t[0][2] * sus->w1t[2][0]) * sus->rho1[0] + (sus->w1t[0][0] * sus->w1t[0][1] + sus->w1t[0][1] * sus->w1t[1][1] + sus->w1t[0][2] * sus->w1t[2][1]) * sus->rho1[1] + (sus->w1t[0][0] * sus->w1t[0][2] + sus->w1t[0][1] * sus->w1t[1][2] + sus->w1t[0][2] * sus->w1t[2][2]) * sus->rho1[2];
	sus->ddr1c[1] = sus->dw1t[1][0] * sus->rho1[0] + sus->dw1t[1][1] * sus->rho1[1] + sus->dw1t[1][2] * sus->rho1[2] + sus->ddr1[1] + (sus->w1t[1][0] * sus->w1t[0][0] + sus->w1t[1][1] * sus->w1t[1][0] + sus->w1t[1][2] * sus->w1t[2][0]) * sus->rho1[0] + (sus->w1t[0][1] * sus->w1t[1][0] + pow(sus->w1t[1][1], 0.2e1) + sus->w1t[1][2] * sus->w1t[2][1]) * sus->rho1[1] + (sus->w1t[1][0] * sus->w1t[0][2] + sus->w1t[1][1] * sus->w1t[1][2] + sus->w1t[1][2] * sus->w1t[2][2]) * sus->rho1[2];
	sus->ddr1c[2] = sus->dw1t[2][0] * sus->rho1[0] + sus->dw1t[2][1] * sus->rho1[1] + sus->dw1t[2][2] * sus->rho1[2] + sus->ddr1[2] + (sus->w1t[2][0] * sus->w1t[0][0] + sus->w1t[2][1] * sus->w1t[1][0] + sus->w1t[2][2] * sus->w1t[2][0]) * sus->rho1[0] + (sus->w1t[2][0] * sus->w1t[0][1] + sus->w1t[2][1] * sus->w1t[1][1] + sus->w1t[2][2] * sus->w1t[2][1]) * sus->rho1[1] + (sus->w1t[0][2] * sus->w1t[2][0] + sus->w1t[1][2] * sus->w1t[2][1] + pow(sus->w1t[2][2], 0.2e1)) * sus->rho1[2];

}