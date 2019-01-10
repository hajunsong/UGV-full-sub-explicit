#include "UnmannedGroundVehicle.h"

void UnmannedGroundVehicle::acceleration_state_suspension(suspension *sus) {
	for (int j = 0; j < 6; j++) {
		sus->dY1h[j] = chs->dY0h[j] + sus->B1[j] * sus->ddq1 + sus->D1[j];
	}

	mat61T61(sus->Myq, chs->dY0h, &sus->ddq1);
	sus->ddq1 = sus->inv_Mqq * (sus->Pq - sus->ddq1);
}