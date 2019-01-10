#include "UnmannedGroundVehicle.h"

void UnmannedGroundVehicle::qdqddq2var() {
	for (int j = 0; j < 3; j++) {
		chs->r0[j] = q[j];
		chs->pi0[j] = q[j + 3];

		chs->dr0[j] = q_dot[j];
		chs->w0[j] = q_dot[j + 3];

		chs->ddr0[j] = q_ddot[j];
		chs->dw0[j] = q_ddot[j + 3];

	}

	// chs->dp0 = 0.5*chs->E0'*chs->w0;
	chs->dp0[0] = 0.5 * chs->E0[0][0] * chs->w0[0] + 0.5 * chs->E0[1][0] * chs->w0[1] + 0.5 * chs->E0[2][0] * chs->w0[2];
	chs->dp0[1] = 0.5 * chs->E0[0][1] * chs->w0[0] + 0.5 * chs->E0[1][1] * chs->w0[1] + 0.5 * chs->E0[2][1] * chs->w0[2];
	chs->dp0[2] = 0.5 * chs->E0[0][2] * chs->w0[0] + 0.5 * chs->E0[1][2] * chs->w0[1] + 0.5 * chs->E0[2][2] * chs->w0[2];
	chs->dp0[3] = 0.5 * chs->E0[0][3] * chs->w0[0] + 0.5 * chs->E0[1][3] * chs->w0[1] + 0.5 * chs->E0[2][3] * chs->w0[2];

	// chs->ddp0 = 0.5*chs->E0'*chs->dw0 - 0.25*(chs->w0'*chs->w0)*chs->p0;
	chs->ddp0[0] = 0.5e0 * chs->E0[0][0] * chs->dw0[0] + 0.5e0 * chs->E0[1][0] * chs->dw0[1] + 0.5e0 * chs->E0[2][0] * chs->dw0[2] - 0.25e0 * (pow(chs->w0[0], 0.2e1) + pow(chs->w0[1], 0.2e1) + pow(chs->w0[2], 0.2e1)) * chs->p0[0];
	chs->ddp0[1] = 0.5e0 * chs->E0[0][1] * chs->dw0[0] + 0.5e0 * chs->E0[1][1] * chs->dw0[1] + 0.5e0 * chs->E0[2][1] * chs->dw0[2] - 0.25e0 * (pow(chs->w0[0], 0.2e1) + pow(chs->w0[1], 0.2e1) + pow(chs->w0[2], 0.2e1)) * chs->p0[1];
	chs->ddp0[2] = 0.5e0 * chs->E0[0][2] * chs->dw0[0] + 0.5e0 * chs->E0[1][2] * chs->dw0[1] + 0.5e0 * chs->E0[2][2] * chs->dw0[2] - 0.25e0 * (pow(chs->w0[0], 0.2e1) + pow(chs->w0[1], 0.2e1) + pow(chs->w0[2], 0.2e1)) * chs->p0[2];
	chs->ddp0[3] = 0.5e0 * chs->E0[0][3] * chs->dw0[0] + 0.5e0 * chs->E0[1][3] * chs->dw0[1] + 0.5e0 * chs->E0[2][3] * chs->dw0[2] - 0.25e0 * (pow(chs->w0[0], 0.2e1) + pow(chs->w0[1], 0.2e1) + pow(chs->w0[2], 0.2e1)) * chs->p0[3];

}