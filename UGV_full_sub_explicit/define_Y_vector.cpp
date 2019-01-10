#include "UnmannedGroundVehicle.h"

void UnmannedGroundVehicle::define_Y_vector() {
	memcpy(Y, chs->r0, sizeof(double) * 3);
	memcpy(Y + 3, chs->p0, sizeof(double) * 4);

	Y[7] = sus[RF].q1;
	Y[8] = sus[RM].q1;
	Y[9] = sus[RR].q1;
	Y[10] = sus[LF].q1;
	Y[11] = sus[LM].q1;
	Y[12] = sus[LR].q1;

	memcpy(Y + 13, chs->dr0, sizeof(double) * 3);
	memcpy(Y + 16, chs->w0, sizeof(double) * 3);

	Y[19] = sus[RF].dq1;
	Y[20] = sus[RM].dq1;
	Y[21] = sus[RR].dq1;
	Y[22] = sus[LF].dq1;
	Y[23] = sus[LM].dq1;
	Y[24] = sus[LR].dq1;

	Y[25] = sus[RF].w_wh;
	Y[26] = sus[RM].w_wh;
	Y[27] = sus[RR].w_wh;
	Y[28] = sus[LF].w_wh;
	Y[29] = sus[LM].w_wh;
	Y[30] = sus[LR].w_wh;
}