#include "UnmannedGroundVehicle.h"

void UnmannedGroundVehicle::var2qdqddq() {
	q[0] = chs->r0[0];
	q[1] = chs->r0[1];
	q[2] = chs->r0[2];
	q[3] = chs->pi0[0];
	q[4] = chs->pi0[1];
	q[5] = chs->pi0[2];

	q_dot[0] = chs->dr0[0];
	q_dot[1] = chs->dr0[1];
	q_dot[2] = chs->dr0[2];
	q_dot[3] = chs->w0[0];
	q_dot[4] = chs->w0[1];
	q_dot[5] = chs->w0[2];

	q_ddot[0] = chs->ddr0[0];
	q_ddot[1] = chs->ddr0[1];
	q_ddot[2] = chs->ddr0[2];
	q_ddot[3] = chs->dw0[0];
	q_ddot[4] = chs->dw0[1];
	q_ddot[5] = chs->dw0[2];
}