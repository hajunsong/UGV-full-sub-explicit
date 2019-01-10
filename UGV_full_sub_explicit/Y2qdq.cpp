#include "UnmannedGroundVehicle.h"

void UnmannedGroundVehicle::Y2qdq() {
	memcpy(chs->r0, Y, sizeof(double) * 3);
	memcpy(chs->p0, Y + 3, sizeof(double) * 4);

	sus[RF].q1 = Y[7];
	sus[RM].q1 = Y[8];
	sus[RR].q1 = Y[9];
	sus[LF].q1 = Y[10];
	sus[LM].q1 = Y[11];
	sus[LR].q1 = Y[12];

	memcpy(chs->dr0, Y + 13, sizeof(double) * 3);
	memcpy(chs->w0, Y + 16, sizeof(double) * 3);

	sus[RF].dq1 = Y[19];
	sus[RM].dq1 = Y[20];
	sus[RR].dq1 = Y[21];
	sus[LF].dq1 = Y[22];
	sus[LM].dq1 = Y[23];
	sus[LR].dq1 = Y[24];

	sus[RF].w_wh = Y[25];
	sus[RM].w_wh = Y[26];
	sus[RR].w_wh = Y[27];
	sus[LF].w_wh = Y[28];
	sus[LM].w_wh = Y[29];
	sus[LR].w_wh = Y[30];
}