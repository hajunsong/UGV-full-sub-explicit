#include "UnmannedGroundVehicle.h"

void UnmannedGroundVehicle::equilibrium() {
	const int n = 31;
	double tol_acc, acc_flag;

	t_current = 0; // simulation time 초기화
	acc_flag;		// 챠량 가속도 값 저장 변수 초기화
	tol_acc = 0.01;		// 평형 상태 판단 기준 값

	intcount = 1;	// absh3 integrator variable

	simulation_flag = 1;	// 평형 상태 해석 시뮬레이션을 구분하는 flag 변수

	file_name = dir + "/EX_sub_" + to_string((int)(h * 1000)) + "ms_Equ_C.csv";

	fopen_s(&fp, file_name.c_str(), "w+");

	while (1) {
		Y2qdq();

		// 동역학 해석 sub main function
		analysis();

		dqddq2Yp();

		// 차량 가속도 값 계산 (3축 선형 가속도 합)
		acc_flag = sqrt(Yp[13] * Yp[13] + Yp[14] * Yp[14] + Yp[15] * Yp[15]);

		// 차량 가속도가 기준 값 이하일 경우 시뮬레이션 종료
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

	// 평형 해석 종료 후 차량의 position 값을 저장
	memcpy(Y_equil, Y, sizeof(double) * 13);
}