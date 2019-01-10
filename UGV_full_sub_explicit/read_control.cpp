#include "UnmannedGroundVehicle.h"

void UnmannedGroundVehicle::read_control() {
	// ���� 1ȸ�� ����Ǿ� �� �������� �ʱ�ȭ ������
	ctrl->e_v_sum = 0;
	ctrl->M_d_hat = 0;
	ctrl->yaw_hat = 0;
	ctrl->WP_size = input->WaypointSize;
	for (int i = 0; i < 10; i++) {
		for (int j = 0; j < 6; j++) {
			ctrl->F_x_error[i][j] = 0;
			ctrl->F_x_error_sum[j] = 0;
		}
	}
}