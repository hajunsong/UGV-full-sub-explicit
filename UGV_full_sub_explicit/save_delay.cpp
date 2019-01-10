#include "UnmannedGroundVehicle.h"

void UnmannedGroundVehicle::save_delay() {
	memcpy(q_delay, q, sizeof(double) * 6);
	memcpy(q_dot_delay, q_dot, sizeof(double) * 6);
	memcpy(q_ddot_delay, q_ddot, sizeof(double) * 6);
	memcpy(chs->p0_delay, chs->p0, sizeof(double) * 4);
	memcpy(chs->dp0_delay, chs->dp0, sizeof(double) * 4);
	memcpy(chs->ddp0_delay, chs->ddp0, sizeof(double) * 4);
	memcpy(chs->L0_delay, chs->L0, sizeof(double) * 6);
	memcpy(chs->dY0h_delay, chs->dY0h, sizeof(double) * 6);
	memcpy(chs->Y0h_delay, chs->Y0h, sizeof(double) * 6);

	for (int i = 0; i < subsystems; i++) {
		sus[i].Pq_delay = sus[i].Pq;
		sus[i].q1_delay = sus[i].q1;
		sus[i].dq1_delay = sus[i].dq1;
		sus[i].ddq1_delay = sus[i].ddq1;
		for (int j = 0; j < 6; j++) {
			sus[i].Py_delay[j] = sus[i].Py[j];
		}
	}
}