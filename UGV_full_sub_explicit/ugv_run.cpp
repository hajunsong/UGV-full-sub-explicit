#include "UnmannedGroundVehicle.h"

void UnmannedGroundVehicle::run() {
	const int n = 31;

	memset(Y, 0, sizeof(double)*n);	// state vector 0���� �ʱ�ȭ

	memcpy(Y, Y_equil, sizeof(double) * 13);

	ctrl->v_d_init = 5;
	Y[13] = ctrl->v_d_init;
	Y[25] = ctrl->v_d_init / tire->PNU_get_unloaded_radius();
	Y[26] = ctrl->v_d_init / tire->PNU_get_unloaded_radius();
	Y[27] = ctrl->v_d_init / tire->PNU_get_unloaded_radius();
	Y[28] = ctrl->v_d_init / tire->PNU_get_unloaded_radius();
	Y[29] = ctrl->v_d_init / tire->PNU_get_unloaded_radius();
	Y[30] = ctrl->v_d_init / tire->PNU_get_unloaded_radius();


	// ���� ��� �����Ϳ� ���� ���� �ڼ� ��ȯ���� ���� �ӵ� ������ ��ȯ �� ���
	double temp[3];
	temp[0] = Y[13]; 
	temp[1] = Y[14]; 
	temp[2] = Y[15];
	mat3331(chs->A0, temp, Y + 13);

	// ����� ��� ���� �ʱ�ȭ
	ctrl->e_v_sum = 0;
	ctrl->M_d_hat = 0;
	ctrl->yaw_hat = 0;
	ctrl->WP_indx = 1;

	t_current = 0;
	intcount = 1;

	simulation_flag = 2;

	file_name = dir + "/EX_sub_" + to_string((int)(h * 1000)) + "ms_Run_C.csv";

	fopen_s(&fp, file_name.c_str(), "w+");

	while (1) {
		Y2qdq();

		analysis();

		dqddq2Yp();

		// explicit integrator
		absh3(h, n);

		if (intcount == 2 || intcount == 6 || intcount >= 8) {
			cout << "Run/Simulation Time : " << to_string(t_current) << ", v_x : " << to_string(ctrl->v_x) << ", Waypoint : " << to_string(ctrl->WP_indx) << endl;
			save_data();
		}

		// simulation time update
		t_current = t_next;
		memcpy(Y, Y_next, sizeof(double)*n);

		// way point ���� ����� �ؼ� ���� ����
		if (ctrl->WP_indx >= ctrl->WP_size - 10) break;
	}

	fclose(fp);
}