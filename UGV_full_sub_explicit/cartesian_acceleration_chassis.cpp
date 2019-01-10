#include "UnmannedGroundVehicle.h"

void UnmannedGroundVehicle::cartesian_acceleration_chassis() {
	memset(chs->dT0, 0, sizeof(double) * 6 * 6);
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			chs->dT0[i][j + 3] = -chs->dr0t[i][j];
		}
	}

	for (int i = 0; i < 6; i++) {
		chs->dY0b[i] = 0;
		for (int j = 0; j < 6; j++) {
			chs->dY0b[i] += chs->dT0[i][j] * chs->Y0h[j] + chs->T0[i][j] * chs->dY0h[j];
		}
	}
	memcpy(chs->ddr0, chs->dY0b, sizeof(double) * 3);
	memcpy(chs->dw0, chs->dY0b + 3, sizeof(double) * 3);

	tilde(chs->dw0, chs->dw0t);

	chs->ddr0c[0] = chs->dw0t[0][0] * chs->rho0[0] + chs->dw0t[0][1] * chs->rho0[1] + chs->dw0t[0][2] * chs->rho0[2] + chs->ddr0[0] + (pow(chs->w0t[0][0], 0.2e1) + chs->w0t[0][1] * chs->w0t[1][0] + chs->w0t[0][2] * chs->w0t[2][0]) * chs->rho0[0] + (chs->w0t[0][0] * chs->w0t[0][1] + chs->w0t[0][1] * chs->w0t[1][1] + chs->w0t[0][2] * chs->w0t[2][1]) * chs->rho0[1] + (chs->w0t[0][0] * chs->w0t[0][2] + chs->w0t[0][1] * chs->w0t[1][2] + chs->w0t[0][2] * chs->w0t[2][2]) * chs->rho0[2];
	chs->ddr0c[1] = chs->dw0t[1][0] * chs->rho0[0] + chs->dw0t[1][1] * chs->rho0[1] + chs->dw0t[1][2] * chs->rho0[2] + chs->ddr0[1] + (chs->w0t[1][0] * chs->w0t[0][0] + chs->w0t[1][1] * chs->w0t[1][0] + chs->w0t[1][2] * chs->w0t[2][0]) * chs->rho0[0] + (chs->w0t[0][1] * chs->w0t[1][0] + pow(chs->w0t[1][1], 0.2e1) + chs->w0t[1][2] * chs->w0t[2][1]) * chs->rho0[1] + (chs->w0t[1][0] * chs->w0t[0][2] + chs->w0t[1][1] * chs->w0t[1][2] + chs->w0t[1][2] * chs->w0t[2][2]) * chs->rho0[2];
	chs->ddr0c[2] = chs->dw0t[2][0] * chs->rho0[0] + chs->dw0t[2][1] * chs->rho0[1] + chs->dw0t[2][2] * chs->rho0[2] + chs->ddr0[2] + (chs->w0t[2][0] * chs->w0t[0][0] + chs->w0t[2][1] * chs->w0t[1][0] + chs->w0t[2][2] * chs->w0t[2][0]) * chs->rho0[0] + (chs->w0t[2][0] * chs->w0t[0][1] + chs->w0t[2][1] * chs->w0t[1][1] + chs->w0t[2][2] * chs->w0t[2][1]) * chs->rho0[1] + (chs->w0t[0][2] * chs->w0t[2][0] + chs->w0t[1][2] * chs->w0t[2][1] + pow(chs->w0t[2][2], 0.2e1)) * chs->rho0[2];

	chs->ddp0[0] = 0.5e0 * chs->E0[0][0] * chs->dw0[0] + 0.5e0 * chs->E0[1][0] * chs->dw0[1] + 0.5e0 * chs->E0[2][0] * chs->dw0[2] - 0.25e0 * (pow(chs->w0[0], 0.2e1) + pow(chs->w0[1], 0.2e1) + pow(chs->w0[2], 0.2e1)) * chs->p0[0];
	chs->ddp0[1] = 0.5e0 * chs->E0[0][1] * chs->dw0[0] + 0.5e0 * chs->E0[1][1] * chs->dw0[1] + 0.5e0 * chs->E0[2][1] * chs->dw0[2] - 0.25e0 * (pow(chs->w0[0], 0.2e1) + pow(chs->w0[1], 0.2e1) + pow(chs->w0[2], 0.2e1)) * chs->p0[1];
	chs->ddp0[2] = 0.5e0 * chs->E0[0][2] * chs->dw0[0] + 0.5e0 * chs->E0[1][2] * chs->dw0[1] + 0.5e0 * chs->E0[2][2] * chs->dw0[2] - 0.25e0 * (pow(chs->w0[0], 0.2e1) + pow(chs->w0[1], 0.2e1) + pow(chs->w0[2], 0.2e1)) * chs->p0[2];
	chs->ddp0[3] = 0.5e0 * chs->E0[0][3] * chs->dw0[0] + 0.5e0 * chs->E0[1][3] * chs->dw0[1] + 0.5e0 * chs->E0[2][3] * chs->dw0[2] - 0.25e0 * (pow(chs->w0[0], 0.2e1) + pow(chs->w0[1], 0.2e1) + pow(chs->w0[2], 0.2e1)) * chs->p0[3];
}