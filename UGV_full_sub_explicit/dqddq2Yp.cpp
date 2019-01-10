#include "UnmannedGroundVehicle.h"

void UnmannedGroundVehicle::dqddq2Yp() {
	memcpy(Yp, chs->dr0, sizeof(double) * 3);
	memcpy(Yp + 3, chs->dp0, sizeof(double) * 4);

	Yp[7] = sus[RF].dq1;
	Yp[8] = sus[RM].dq1;
	Yp[9] = sus[RR].dq1;
	Yp[10] = sus[LF].dq1;
	Yp[11] = sus[LM].dq1;
	Yp[12] = sus[LR].dq1;

	memcpy(Yp + 13, chs->ddr0, sizeof(double) * 3);
	memcpy(Yp + 16, chs->dw0, sizeof(double) * 3);

	Yp[19] = sus[RF].ddq1;
	Yp[20] = sus[RM].ddq1;
	Yp[21] = sus[RR].ddq1;
	Yp[22] = sus[LF].ddq1;
	Yp[23] = sus[LM].ddq1;
	Yp[24] = sus[LR].ddq1;

	Yp[25] = sus[RF].dw_wh;
	Yp[26] = sus[RM].dw_wh;
	Yp[27] = sus[RR].dw_wh;
	Yp[28] = sus[LF].dw_wh;
	Yp[29] = sus[LM].dw_wh;
	Yp[30] = sus[LR].dw_wh;
}