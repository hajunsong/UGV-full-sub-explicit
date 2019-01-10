#include "UnmannedGroundVehicle.h"

void UnmannedGroundVehicle::wheel_spin_dyn() {
	// 제어기 계산 결과인 토크를 index를 정렬해서 저장
	sus[RF].T_in = ctrl->motor_torque[1];
	sus[RM].T_in = ctrl->motor_torque[3];
	sus[RR].T_in = ctrl->motor_torque[5];
	sus[LF].T_in = ctrl->motor_torque[0];
	sus[LM].T_in = ctrl->motor_torque[2];
	sus[LR].T_in = ctrl->motor_torque[4];

	double Iyy = 21;   // 회전 축 관성 모멘트

	sus[RF].dw_wh = (sus[RF].T_in - sus[RF].Fx*sus[RF].R_d) / Iyy;
	sus[RM].dw_wh = (sus[RM].T_in - sus[RM].Fx*sus[RM].R_d) / Iyy;
	sus[RR].dw_wh = (sus[RR].T_in - sus[RR].Fx*sus[RR].R_d) / Iyy;
	sus[LF].dw_wh = (sus[LF].T_in - sus[LF].Fx*sus[LF].R_d) / Iyy;
	sus[LM].dw_wh = (sus[LM].T_in - sus[LM].Fx*sus[LM].R_d) / Iyy;
	sus[LR].dw_wh = (sus[LR].T_in - sus[LR].Fx*sus[LR].R_d) / Iyy;
}