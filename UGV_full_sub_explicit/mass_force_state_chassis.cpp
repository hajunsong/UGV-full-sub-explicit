#include "UnmannedGroundVehicle.h"

void UnmannedGroundVehicle::mass_force_state_chassis() {
	double A0_C00[3][3], A0_C00_J0p[3][3];

	mat3333(chs->A0, chs->C00, A0_C00);
	mat3333(A0_C00, chs->J0p, A0_C00_J0p);
	mat3333T(A0_C00_J0p, A0_C00, chs->J0c);

	tilde(chs->r0c, chs->r0ct);
	tilde(chs->dr0c, chs->dr0ct);
	tilde(chs->w0, chs->w0t);

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			chs->M0h[i][j] = i == j ? chs->m0 : 0;
			chs->M0h[i][j + 3] = -chs->m0*chs->r0ct[i][j];
			chs->M0h[i + 3][j] = chs->m0*chs->r0ct[i][j];
			chs->M0h[i + 3][j + 3] = 0;
			for (int k = 0; k < 3; k++) {
				chs->M0h[i + 3][j + 3] += chs->r0t[i][k] * chs->r0t[k][j];
			}
			chs->M0h[i + 3][j + 3] *= -chs->m0;
			chs->M0h[i + 3][j + 3] += chs->J0c[i][j];
		}
	}

	chs->F0c[0] = 0;
	chs->F0c[1] = 0;
	chs->F0c[2] = chs->m0*g;
	chs->T0c[0] = 0;
	chs->T0c[1] = 0;
	chs->T0c[2] = 0;

	chs->Q0h_g[0] = chs->m0 * chs->dr0ct[0][0] * chs->w0[0] + chs->m0 * chs->dr0ct[0][1] * chs->w0[1] + chs->m0 * chs->dr0ct[0][2] * chs->w0[2] + chs->F0c[0];
	chs->Q0h_g[1] = chs->m0 * chs->dr0ct[1][0] * chs->w0[0] + chs->m0 * chs->dr0ct[1][1] * chs->w0[1] + chs->m0 * chs->dr0ct[1][2] * chs->w0[2] + chs->F0c[1];
	chs->Q0h_g[2] = chs->m0 * chs->dr0ct[2][0] * chs->w0[0] + chs->m0 * chs->dr0ct[2][1] * chs->w0[1] + chs->m0 * chs->dr0ct[2][2] * chs->w0[2] + chs->F0c[2];
	chs->Q0h_g[3] = chs->r0ct[0][0] * chs->F0c[0] + chs->r0ct[0][1] * chs->F0c[1] + chs->r0ct[0][2] * chs->F0c[2] + chs->T0c[0] + (chs->m0 * chs->r0ct[0][0] * chs->dr0ct[0][0] + chs->m0 * chs->r0ct[0][1] * chs->dr0ct[1][0] + chs->m0 * chs->r0ct[0][2] * chs->dr0ct[2][0]) * chs->w0[0] + (chs->m0 * chs->r0ct[0][0] * chs->dr0ct[0][1] + chs->m0 * chs->r0ct[0][1] * chs->dr0ct[1][1] + chs->m0 * chs->r0ct[0][2] * chs->dr0ct[2][1]) * chs->w0[1] + (chs->m0 * chs->r0ct[0][0] * chs->dr0ct[0][2] + chs->m0 * chs->r0ct[0][1] * chs->dr0ct[1][2] + chs->m0 * chs->r0ct[0][2] * chs->dr0ct[2][2]) * chs->w0[2] - (chs->w0t[0][0] * chs->J0c[0][0] + chs->w0t[0][1] * chs->J0c[1][0] + chs->w0t[0][2] * chs->J0c[2][0]) * chs->w0[0] - (chs->w0t[0][0] * chs->J0c[0][1] + chs->w0t[0][1] * chs->J0c[1][1] + chs->w0t[0][2] * chs->J0c[2][1]) * chs->w0[1] - (chs->w0t[0][0] * chs->J0c[0][2] + chs->w0t[0][1] * chs->J0c[1][2] + chs->w0t[0][2] * chs->J0c[2][2]) * chs->w0[2];
	chs->Q0h_g[4] = chs->r0ct[1][0] * chs->F0c[0] + chs->r0ct[1][1] * chs->F0c[1] + chs->r0ct[1][2] * chs->F0c[2] + chs->T0c[1] + (chs->m0 * chs->r0ct[1][0] * chs->dr0ct[0][0] + chs->m0 * chs->r0ct[1][1] * chs->dr0ct[1][0] + chs->m0 * chs->r0ct[1][2] * chs->dr0ct[2][0]) * chs->w0[0] + (chs->m0 * chs->r0ct[1][0] * chs->dr0ct[0][1] + chs->m0 * chs->r0ct[1][1] * chs->dr0ct[1][1] + chs->m0 * chs->r0ct[1][2] * chs->dr0ct[2][1]) * chs->w0[1] + (chs->m0 * chs->r0ct[1][0] * chs->dr0ct[0][2] + chs->m0 * chs->r0ct[1][1] * chs->dr0ct[1][2] + chs->m0 * chs->r0ct[1][2] * chs->dr0ct[2][2]) * chs->w0[2] - (chs->w0t[1][0] * chs->J0c[0][0] + chs->w0t[1][1] * chs->J0c[1][0] + chs->w0t[1][2] * chs->J0c[2][0]) * chs->w0[0] - (chs->w0t[1][0] * chs->J0c[0][1] + chs->w0t[1][1] * chs->J0c[1][1] + chs->w0t[1][2] * chs->J0c[2][1]) * chs->w0[1] - (chs->w0t[1][0] * chs->J0c[0][2] + chs->w0t[1][1] * chs->J0c[1][2] + chs->w0t[1][2] * chs->J0c[2][2]) * chs->w0[2];
	chs->Q0h_g[5] = chs->r0ct[2][0] * chs->F0c[0] + chs->r0ct[2][1] * chs->F0c[1] + chs->r0ct[2][2] * chs->F0c[2] + chs->T0c[2] + (chs->m0 * chs->r0ct[2][0] * chs->dr0ct[0][0] + chs->m0 * chs->r0ct[2][1] * chs->dr0ct[1][0] + chs->m0 * chs->r0ct[2][2] * chs->dr0ct[2][0]) * chs->w0[0] + (chs->m0 * chs->r0ct[2][0] * chs->dr0ct[0][1] + chs->m0 * chs->r0ct[2][1] * chs->dr0ct[1][1] + chs->m0 * chs->r0ct[2][2] * chs->dr0ct[2][1]) * chs->w0[1] + (chs->m0 * chs->r0ct[2][0] * chs->dr0ct[0][2] + chs->m0 * chs->r0ct[2][1] * chs->dr0ct[1][2] + chs->m0 * chs->r0ct[2][2] * chs->dr0ct[2][2]) * chs->w0[2] - (chs->w0t[2][0] * chs->J0c[0][0] + chs->w0t[2][1] * chs->J0c[1][0] + chs->w0t[2][2] * chs->J0c[2][0]) * chs->w0[0] - (chs->w0t[2][0] * chs->J0c[0][1] + chs->w0t[2][1] * chs->J0c[1][1] + chs->w0t[2][2] * chs->J0c[2][1]) * chs->w0[1] - (chs->w0t[2][0] * chs->J0c[0][2] + chs->w0t[2][1] * chs->J0c[1][2] + chs->w0t[2][2] * chs->J0c[2][2]) * chs->w0[2];
}