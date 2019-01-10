#include "UnmannedGroundVehicle.h"

void UnmannedGroundVehicle::equilibrium() {
	const int n = 31;
	double tol_acc, acc_flag;

	t_current = 0; // simulation time �ʱ�ȭ
	acc_flag;		// í�� ���ӵ� �� ���� ���� �ʱ�ȭ
	tol_acc = 0.01;		// ���� ���� �Ǵ� ���� ��

	intcount = 1;	// absh3 integrator variable

	simulation_flag = 1;	// ���� ���� �ؼ� �ùķ��̼��� �����ϴ� flag ����

	file_name = dir + "/EX_sub_" + to_string((int)(h * 1000)) + "ms_Equ_C.csv";

	fopen_s(&fp, file_name.c_str(), "w+");

	while (1) {
		Y2qdq();

		// ������ �ؼ� sub main function
		analysis();

		dqddq2Yp();

		// ���� ���ӵ� �� ��� (3�� ���� ���ӵ� ��)
		acc_flag = sqrt(Yp[13] * Yp[13] + Yp[14] * Yp[14] + Yp[15] * Yp[15]);

		// ���� ���ӵ��� ���� �� ������ ��� �ùķ��̼� ����
		if (acc_flag < tol_acc) break;

		// explicit integrator
		absh3(h, n);

		if (intcount == 2 || intcount == 6 || intcount >= 8) {
			save_data();
			cout << "Equ/Simulation Time : " << to_string(t_current) << ", Acceleration : " << to_string(acc_flag) << endl;
		}
		// simulation time update
		t_current = t_next;
		memcpy(Y, Y_next, sizeof(double)*n);
	}

	fclose(fp);

	// ���� �ؼ� ���� �� ������ position ���� ����
	memcpy(Y_equil, Y, sizeof(double) * 13);
}