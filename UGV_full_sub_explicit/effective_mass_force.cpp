#include "UnmannedGroundVehicle.h"

void UnmannedGroundVehicle::effective_mass_force(suspension *sus) {
	double K1_B1[6], B1_Myq = 0, B1_Py = 0, Myq_MyqT[6][6];

	for (int j = 0; j < 6; j++) {
		sus->L1[j] = sus->Q1h_g[j] + sus->Q1h_TSDA[j];
	}

	for (int j = 0; j < 6; j++) {
		sus->L1[j] = sus->Q1h_g[j] + sus->Q1h_TSDA[j];
		for (int k = 0; k < 6; k++) {
			sus->K1[j][k] = sus->M1h[j][k];
			sus->Myy[j][k] = sus->K1[j][k];
		}
	}

	mat6661(sus->K1, sus->B1, K1_B1);

	for (int j = 0; j < 6; j++) {
		sus->Myq[j] = K1_B1[j];
	}

	mat61T61(sus->B1, sus->Myq, &B1_Myq);
	sus->Mqq = B1_Myq;

	mat6661(sus->K1, sus->D1, sus->K1_D1);
	for (int j = 0; j < 6; j++) {
		sus->Py[j] = sus->L1[j] - sus->K1_D1[j];
	}

	mat61T61(sus->B1, sus->Py, &B1_Py);
	sus->Pq = B1_Py;

	sus->inv_Mqq = 1 / sus->Mqq;
	mat6161T(sus->Myq, sus->Myq, Myq_MyqT);
	for (int j = 0; j < 6; j++) {
		for (int k = 0; k < 6; k++) {
			Myq_MyqT[j][k] = sus->inv_Mqq * Myq_MyqT[j][k];
		}
	}

	for (int j = 0; j < 6; j++) {
		for (int k = 0; k < 6; k++) {
			sus->Mhc[j][k] = sus->Myy[j][k] - Myq_MyqT[j][k];
		}
	}

	for (int j = 0; j < 6; j++) {
		sus->Qhc[j] = sus->Myq[j] * sus->inv_Mqq * sus->Pq;
	}

	for (int j = 0; j < 6; j++) {
		sus->Qhc[j] = sus->Py[j] - sus->Qhc[j];
	}
}