#include "UnmannedGroundVehicle.h"

void UnmannedGroundVehicle::save_data() {
	// 애니메이션 상에서 타이어의 회전을 표현하기 위해 angle 로 적분하는 부분
	if (t_current == 0) {
		sus[RF].theta_wh = 0;
		sus[RM].theta_wh = 0;
		sus[RR].theta_wh = 0;
		sus[LF].theta_wh = 0;
		sus[LM].theta_wh = 0;
		sus[LR].theta_wh = 0;
	}
	else {
		sus[RF].theta_wh = sus[RF].theta_wh + h * sus[RF].w_wh + 0.5*h*h*sus[RF].dw_wh;
		sus[RM].theta_wh = sus[RM].theta_wh + h * sus[RM].w_wh + 0.5*h*h*sus[RM].dw_wh;
		sus[RR].theta_wh = sus[RR].theta_wh + h * sus[RR].w_wh + 0.5*h*h*sus[RR].dw_wh;
		sus[LF].theta_wh = sus[LF].theta_wh + h * sus[LF].w_wh + 0.5*h*h*sus[LF].dw_wh;
		sus[LM].theta_wh = sus[LM].theta_wh + h * sus[LM].w_wh + 0.5*h*h*sus[LM].dw_wh;
		sus[LR].theta_wh = sus[LR].theta_wh + h * sus[LR].w_wh + 0.5*h*h*sus[LR].dw_wh;
	}
	if (t_current >= 0 && t_current <= 0) {
		fprintf(fp, "Time,Position_x,Position_y,Position_z,");
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				fprintf(fp, "A0_%d%d,", i, j);
			}
		}
		fprintf(fp, "sus_RF, sus_RM, sus_RR, sus_LF, sus_LM, sus_LR,");
		fprintf(fp, "wheel_RF, wheel_RM, wheel_RR, wheel_LF, wheel_LM, wheel_LR,");
		fprintf(fp, "Roll, Pitch, Yaw,");
		fprintf(fp, "Velocity_Body_x, Velocity_Body_y, Velocity_Body_z,");
		fprintf(fp, "Acceleration_Body_x, Acceleration_Body_y, Acceleration_Body_z,");
		fprintf(fp, "Velocity_Global_x, Velocity_Global_y, Velocity_Global_z,");
		fprintf(fp, "Acceleration_Global_x, Acceleration_Global_y, Acceleration_Global_z,");
		fprintf(fp, "\n");
	}
	fprintf(fp, "%5.5f,", t_current);
	for (int i = 0; i < 3; i++) fprintf(fp, "%10.10f,", chs->r0[i]);
	for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) fprintf(fp, "%10.10f,", chs->A0[i][j]);
	for (int i = 0; i < 6; i++) fprintf(fp, "%10.10f,", sus[i].q1);
	for (int i = 0; i < 6; i++) fprintf(fp, "%10.10f,", sus[i].theta_wh);
	fprintf(fp, "%10.10f,%10.10f,%10.10f,", chs->roll_ang, chs->pitch_ang, chs->yaw_ang);
	for (int i = 0; i < 3; i++) fprintf(fp, "%10.10f,", chs->dr0cp[i]);
	for (int i = 0; i < 3; i++) fprintf(fp, "%10.10f,", chs->ddr0cp[i]);
	for (int i = 0; i < 3; i++) fprintf(fp, "%10.10f,", chs->dr0c[i]);
	for (int i = 0; i < 3; i++) fprintf(fp, "%10.10f,", chs->ddr0c[i]);
	fprintf(fp, "\n");
}