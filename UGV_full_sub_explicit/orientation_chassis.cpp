#include "UnmannedGroundVehicle.h"

void UnmannedGroundVehicle::orientation_chassis() {
	double e0 = chs->p0[0], e1 = chs->p0[1], e2 = chs->p0[2], e3 = chs->p0[3];

	chs->E0[0][0] = -chs->p0[1];	
	chs->E0[0][1] = chs->p0[0];	
	chs->E0[0][2] = -chs->p0[3];	
	chs->E0[0][3] = chs->p0[2];
	chs->E0[1][0] = -chs->p0[2];	
	chs->E0[1][1] = chs->p0[3];	
	chs->E0[1][2] = chs->p0[0];	
	chs->E0[1][3] = -chs->p0[1];
	chs->E0[2][0] = -chs->p0[3];	
	chs->E0[2][1] = -chs->p0[2];	
	chs->E0[2][2] = chs->p0[1];	
	chs->E0[2][3] = chs->p0[0];

	chs->G0[0][0] = -chs->p0[1];	
	chs->G0[0][1] = chs->p0[0];	
	chs->G0[0][2] = chs->p0[3];	
	chs->G0[0][3] = -chs->p0[2];
	chs->G0[1][0] = -chs->p0[2];	
	chs->G0[1][1] = -chs->p0[3];	
	chs->G0[1][2] = chs->p0[0];	
	chs->G0[1][3] = chs->p0[1];
	chs->G0[2][0] = -chs->p0[3];	
	chs->G0[2][1] = chs->p0[2];	
	chs->G0[2][2] = -chs->p0[1];	
	chs->G0[2][3] = chs->p0[0];

	chs->A0[0][0] = 2 * (e0*e0 + e1 * e1 - 0.5);
	chs->A0[0][1] = 2 * (e1*e2 - e0 * e3);
	chs->A0[0][2] = 2 * (e1*e3 + e0 * e2);
	chs->A0[1][0] = 2 * (e1*e2 + e0 * e3);
	chs->A0[1][1] = 2 * (e0*e0 + e2 * e2 - 0.5);
	chs->A0[1][2] = 2 * (e2*e3 - e0 * e1);
	chs->A0[2][0] = 2 * (e1*e3 - e0 * e2);
	chs->A0[2][1] = 2 * (e2*e3 + e0 * e1);
	chs->A0[2][2] = 2 * (e0*e0 + e3 * e3 - 0.5);

	mat34T31(chs->E0, chs->w0, chs->dp0);
	for (int i = 0; i < 4; i++) {
		chs->dp0[i] *= 0.5;
	}

	chs->roll_ang = -atan2(-chs->A0[2][1], chs->A0[2][2]);
	chs->pitch_ang = atan2(chs->A0[2][0], sqrt(chs->A0[2][1] * chs->A0[2][1] + chs->A0[2][2] * chs->A0[2][2]));
	chs->yaw_ang = atan2(chs->A0[1][0], chs->A0[0][0]);
}