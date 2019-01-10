#include "UnmannedGroundVehicle.h"

void UnmannedGroundVehicle::LP_control() {
	/*
	LP_control() : 자율 주행 제어기
	작성자: 홍효성
	Date: 2017.02.17

	input variables
	v_di: desired velocity
	WP : Waypoint data
	Yp : current derivative of state
	step_size: integration step_size

	output variables
	motor_torque: 각 휠에 전달되는 모터 토크 명령

	*/

	////////// Section 0. 차량 파라미터 및 지역 변수 초기화 //////////

	////////// 입력 받아야할 차량 모델 파라미터 //////////
	double m_vehicle = 6762;		// vehicle mass [kg]
	double t_w = 1.948;             // 좌, 우측 휠 간 거리(m) = tread	(변경 2016.11.30 홍효성)	
	double I_z = 13201;             // total moment of inertia of z - axis(kgm ^ 2)
	double tire_radius = tire->PNU_get_unloaded_radius();	// 타이어 반지름
	double torque_limit = 3416;		//[Nm], 기어비: 17.08 기준
									//double torque_limit = 10000;
									////////// 지역 변수 //////////
	int i, j;						// 반복문에 사용하기 위한 변수
	int indx[4] = { 0 };			// lusolve4 (LU decomponent) 함수에서 사용하기 위한 변수
	double M_fac[4][4] = { 0 };		// lusolve4 (LU decomponent) 함수에서 사용하기 위한 변수
	int WP_pv_indx;						// preview point 계산을 위한 waypoint 인덱스
	double x_c, y_c;				// 글로벌 좌표에서 본 차량 무게 중심의 위치
	double yaw, yaw_rate;			//요 각도, 요 각속도
	double L = 1.2;                   // Preview distance 거리 배율(m)
	double F_z[6], F_x[6];			// 수직 방향 타이어력, 종 방향 타이어력
	double W1, W2, W3, W4, W5, W6;	// 타이어력 분배에 사용되는 weight factor	
	double F_x_A[4][4], F_x_B[4], F_x_1234[4], F_xd[6];	// 타이어력 분배에 사용되는 행렬
	double K_vp, K_vi, K_yp, K_yd;	// 제어 게인
	double M_c, M_c_true;			// 요 모멘트 제어 명령 및 실제 타이어력에서 발생되는 요 모멘트
	double T;						// step time
	double l, eta, p;				// 외란 요 모멘트 관측기에 사용되는 게인
	double F_tire_limit;				// 종 방향 타이어력 명령 limit, 
	double limit_ratio;				//모터 토크 명령이 한계 값을 초과할 경우 토크를 비례 배분하여 제한 위한 비율
	double WP_previous[4], WP_next[4];			// WP_previous: 차량이 위치하고 있는 현재 waypoint, WP_next: WP_previous 다음의 waypoint
	double WP_pv_previous[4], WP_pv_next[4];	// preview point 계산을 위한 현재 waypoint 및 다음 waypoint
	double d_wp;						// 각 waypoint 사이의 거리 (WP_previous 부터 WP_next 까지)
	double yaw_wp;					// 현재 소속된 waypoint의 방향(각도)
	double u_wp;					// 현재 소속된 waypoint 내에서 차량의 위치를 상대적으로 나타내기 위한 비율 (0~1까지 범위, u_wp > 1 인 경우 차량이 현재 waypoint를 지나쳤다는 의미)
	double x_n, y_n;				// 현재 소속된 waypoint 벡터 위에서 차량 무게중심에 대한 법선 벡터의 교차점
	double L_pv; // preview distance 
	double d_rest, L_pv_rest;	// preview distance에 의해 계산된 지향점까지의 거리가 해당 waypoint 범위보다 클 경우 초과된 만큼의 거리 (preview waypoint를 update 할 때 사용)
	double x_pv, y_pv, yaw_wp_pv; // preview distance에 의한 지향점 좌표(x_pv, y_pv) 및 차량 무게중심으로부터의 지향 각
	double de_psi;	// 지향각속도 오차 (desired yaw_rate - current yaw_rate)

					// 속도 및 지향각 제어 게인
	K_vp = 10;
	K_vi = 0;
	K_yd = 10;
	K_yp = 30;
	double e_y_k = 40;	// K gain for stanley method
	double vel_offset = 20; // for stanley method

	////////// 경로 추종 제어에 사용하기 위한 차량 상태 변수 입력 //////////

	yaw = chs->yaw_ang;     // 차량의 지향각 (rad)
	yaw_rate = chs->w0[2];  // 지향각속도 (rad/s)	
	x_c = chs->r0c[0];		// 글로벌 좌표에서의 차량 무게중심의 x 좌표
	y_c = chs->r0c[1];		// 글로벌 좌표에서의 차량 무게중심의 y 좌표
	ctrl->v_x = chs->dr0cp[0];		// 차량의 로컬 x 방향 속도(LOCAL)

	// 수직 방향 타이어력 (타이어 모델의 변수로부터 입력)
	F_z[0] = sus[LF].Fz;		// LF (Left, Front)
	F_z[1] = sus[RF].Fz;		// FR (Right, Front)
	F_z[2] = sus[LM].Fz;		// LM (Left, Middle)
	F_z[3] = sus[RM].Fz;		// RM (Right, Middle)
	F_z[4] = sus[LR].Fz;		// LR (Left, Right)
	F_z[5] = sus[RR].Fz;		// RR (Right, Rear)
	//F_z[0] = m_vehicle / 6.0;   // LF (Left, Front)
	//F_z[1] = m_vehicle / 6.0;   // FR (Right, Front)
	//F_z[2] = m_vehicle / 6.0;   // LM (Left, Middle)
	//F_z[3] = m_vehicle / 6.0;   // RM (Right, Middle)
	//F_z[4] = m_vehicle / 6.0;   // LR (Left, Right)
	//F_z[5] = m_vehicle / 6.0;   // RR (Right, Rear)

	// 종 방향 타이어력 (타이어 모델의 변수로부터 입력)
	F_x[0] = sus[LF].Fx;		// LF (Left, Front)
	F_x[1] = sus[RF].Fx;		// FR (Right, Front)
	F_x[2] = sus[LM].Fx;		// LM (Left, Middle)
	F_x[3] = sus[RM].Fx;		// RM (Right, Middle)
	F_x[4] = sus[LR].Fx;		// LR (Left, Right)
	F_x[5] = sus[RR].Fx;		// RR (Right, Rear)

								// 타이어력 분배에 사용되는 weight factor	정의
	W1 = 1; W2 = 1; W3 = 1; W4 = 1; W5 = 1; W6 = 1;

	// 외란 요 모멘트 관측기 게인
	T = h;					// step time (동역학 모델과 동기화)
	l = 10 * I_z;				// observer gain
	eta = 25 * I_z;				// observer gain
	p = l / I_z;				// observer gain

	// 종 방향 타이어력 제한
	F_tire_limit = torque_limit / tire_radius;

	// waypoint 시작 점에 차량 현재 주행 속도 입력 (optimal_velocity_profile에 사용하기 위해 초기화 해줌)
	//WP[0][2][sim->thread_indx] = ctrl->v_x;	

	//VehicleVelocity[0] = ctrl->v_x;

	////////// Section 1. 지향 점 계산 ////////// 
	////////// Input: 차량 현재 위치 (x_c, y_c), Waypoint data (WP)
	////////// Output: 지향각 명령 (yaw_d)

	if (simulation_flag == 2) {										// 병렬 주행 (RTT) 모드인 경우에만 계산 (Equilibrium 모드에서는 계산 안함)
		// 		for (i = 0; i < 4; i++) {
		// 			WP_previous[i] = WP[ctrl->WP_indx - 1][i][sim->thread_indx];			// 현재 차량이 소속된 좌표인 WP_previous에 waypoint의 X, Y 좌표와 주행 속도 명령 그리고 stability indx = 0 할당
		// 			WP_next[i] = WP[ctrl->WP_indx][i][sim->thread_indx];				// 현재 좌표의 다음 좌표인 WP_next에 waypoint의 X, Y 좌표와 주행 속도 명령 그리고 stability indx = 0 할당
		// 		}
		WP_previous[0] = input->LocalPath[ctrl->WP_indx - 1][0];
		WP_previous[1] = input->LocalPath[ctrl->WP_indx - 1][1];
		//WP_previous[2] = VehicleVelocity[ctrl->WP_indx - 1];
		//WP_previous[3] = StabilityIndex[ctrl->WP_indx - 1];
		WP_next[0] = input->LocalPath[ctrl->WP_indx][0];
		WP_next[1] = input->LocalPath[ctrl->WP_indx][1];
		//WP_next[2] = VehicleVelocity[ctrl->WP_indx];
		//WP_next[3] = StabilityIndex[ctrl->WP_indx];


		d_wp = sqrt((WP_next[0] - WP_previous[0])*(WP_next[0] - WP_previous[0]) + (WP_next[1] - WP_previous[1])*(WP_next[1] - WP_previous[1]));		// WP_previous과 WP_next 사이의 거리 계산
		yaw_wp = atan2((WP_next[1] - WP_previous[1]), (WP_next[0] - WP_previous[0]));										// waypoint의 direction(rad) 계산
		u_wp = ((x_c - WP_previous[0])*(WP_next[0] - WP_previous[0]) + (y_c - WP_previous[1])*(WP_next[1] - WP_previous[1])) / (d_wp*d_wp);	// WP_previous과 WP_next 사이에서 차량의 상대 위치를 비율로 계산 (0 ~ 1 범위)

		while (u_wp > 1)													// u_wp > 1 : 차량의 위치가 WP_next 범위를 초과한 경우 -> 새로운 waypoint 할당
		{
			//WP[ctrl->WP_indx][2][sim->thread_indx] = ctrl->v_x;			// 해당 WP_indx 앞에서의 차량 현재 속도 입력
			//VehicleVelocity[ctrl->WP_indx] = ctrl->v_x;

			if (ctrl->WP_indx >= ctrl->WP_size - 1) {						// WP_indx가 마지막 값에 도달하면 더 이상 WP_indx를 업데이트 하지 않음
				break;
			}

			ctrl->WP_indx++;												// WP_previous과 WP_next 좌표를 새로 업데이트하기 위해 WP_indx를 +1 시킨다.
			ctrl->WP_indx_update_flag = 1;									// WP_indx가 업데이트 된 경우 LP_stability_metric()가 실행됐을 때 stability indx 항목을 1로 초기화 시켜준다.

																			// waypoint 새로 할당
																			// 			for (i = 0; i < 4; i++) {
																			// 				WP_previous[i] = WP[ctrl->WP_indx - 1][i][sim->thread_indx];		// 현재 차량이 소속된 좌표인 WP_previous에 waypoint의 X, Y 좌표와 주행 속도 명령 그리고 stability indx = 0 할당
																			// 				WP_next[i] = WP[ctrl->WP_indx][i][sim->thread_indx];			// 현재 차량이 소속된 좌표인 WP_next에 waypoint의 X, Y 좌표와 주행 속도 명령 그리고 stability indx = 0 할당
																			// 			}
			WP_previous[0] = input->LocalPath[ctrl->WP_indx - 1][0];
			WP_previous[1] = input->LocalPath[ctrl->WP_indx - 1][1];
			//WP_previous[2] = VehicleVelocity[ctrl->WP_indx - 1];
			//WP_previous[3] = StabilityIndex[ctrl->WP_indx - 1];
			WP_next[0] = input->LocalPath[ctrl->WP_indx][0];
			WP_next[1] = input->LocalPath[ctrl->WP_indx][1];
			//WP_next[2] = VehicleVelocity[ctrl->WP_indx];
			//WP_next[3] = StabilityIndex[ctrl->WP_indx];

			d_wp = sqrt((WP_next[0] - WP_previous[0])*(WP_next[0] - WP_previous[0]) + (WP_next[1] - WP_previous[1])*(WP_next[1] - WP_previous[1]));		// WP_previous과 WP_next 사이의 거리 계산
			yaw_wp = atan2((WP_next[1] - WP_previous[1]), (WP_next[0] - WP_previous[0]));										// waypoint의 direction(rad) 계산
			u_wp = ((x_c - WP_previous[0])*(WP_next[0] - WP_previous[0]) + (y_c - WP_previous[1])*(WP_next[1] - WP_previous[1])) / (d_wp*d_wp);	// WP_previous과 WP_next 사이에서 차량의 상대 위치를 비율로 계산 (0 ~ 1 범위)
		}

		x_n = WP_previous[0] + u_wp * (WP_next[0] - WP_previous[0]);		// 현재 소속된 waypoint 벡터 위에서 차량 무게중심에 대한 법선 벡터의 교차점 X 좌표
		y_n = WP_previous[1] + u_wp * (WP_next[1] - WP_previous[1]);		// 현재 소속된 waypoint 벡터 위에서 차량 무게중심에 대한 법선 벡터의 교차점 Y 좌표

		// 횡 방향 위치 오차 계산 (lateral position error)
		//if (y_n > y_c) {							// 원래 LPE는 항상 양수로 계산되지만 경로에 대하여 차량 Y 좌표의 상대 위치에 따라 부호가 바뀌어 보이도록 임시 조치함
		//	ctrl->e_l = sqrt((x_n - x_c)*(x_n - x_c) + (y_n - y_c)*(y_n - y_c));		// e_l : 횡 방향 위치 오차
		//}
		//else if (y_n < y_c) {
		//	ctrl->e_l = sqrt((x_n - x_c)*(x_n - x_c) + (y_n - y_c)*(y_n - y_c)) * (-1);
		//}
		//else {
		//	ctrl->e_l = sqrt((x_n - x_c)*(x_n - x_c) + (y_n - y_c)*(y_n - y_c)) * 0;
		//}
		//a2b3 - a3b2, a3b1 - a1b3, a1b2 - a2b1
		double v_car[3] = { x_c - WP_previous[0] , y_c - WP_previous[1] , 0 };
		double v_wp[3] = { WP_next[0] - WP_previous[0] , WP_next[1] - WP_previous[1], 0 };
		double v_car_wp_cross[3] = { v_car[1] * v_wp[2] - v_car[2] * v_wp[1], v_car[2] * v_wp[0] - v_car[0] * v_wp[2], v_car[0] * v_wp[1] - v_car[1] * v_wp[0] };

		ctrl->e_l = sqrt((x_n - x_c)*(x_n - x_c) + (y_n - y_c)*(y_n - y_c))*fsign(v_car_wp_cross[2]);

		// Preview distance
		if (fabs(ctrl->v_x) > L) {		// Preview distance는 차량 주행 속도에 비례하도록 설정함
			L_pv = fabs(ctrl->v_x)*L;
		}
		else {
			L_pv = L;					// 차량 주행 속도가 1 m/s 보다 작을 때는 L_pv = 1 (m) 로 지정함. 여기서 L = 1 이다.
		}

		if (ctrl->WP_indx < ctrl->WP_size) {	// WP_indx가 WP_indx의 마지막 지점에 도달하지 않았을 경우
			WP_pv_indx = ctrl->WP_indx;			// Preview waypoint의 indx를 현재 waypoint의 indx로 동기화
												//d_rest = sqrt((WP[WP_pv_indx][0][sim->thread_indx] - x_n)*(WP[WP_pv_indx][0][sim->thread_indx] - x_n) + (WP[WP_pv_indx][1][sim->thread_indx] - y_n)*(WP[WP_pv_indx][1][sim->thread_indx] - y_n));	// 현재 waypoint의 말단인 WP_next까지 남은 거리 (distance_rest)
			d_rest = sqrt((input->LocalPath[WP_pv_indx][0] - x_n) * (input->LocalPath[WP_pv_indx][0] - x_n) + (input->LocalPath[WP_pv_indx][1] - y_n)*(input->LocalPath[WP_pv_indx][1] - y_n));

			if (L_pv > d_rest) {			// Preview distance가 현재 waypoint의 범위를 초과할 경우
				L_pv_rest = L_pv - d_rest;	// 초과된 만큼을 계산하여 L_pv_rest에 저장
				while (L_pv_rest > 0) {		// 초과된 거리가 양수이면					
					WP_pv_indx = WP_pv_indx + 1;		// Preview waypoint의 indx을 +1 업데이트 시킴 (Preview waypoint 업데이트를 위함)
					if (WP_pv_indx >= ctrl->WP_size) {	// 만약 Preview waypoint의 indx가 전체 waypoint indx 범위를 초과할 경우 마지막 waypoint를 preview point로 할당
														// 						for (i = 0; i < 4; i++) {
														// 							WP_pv_previous[i] = WP[ctrl->WP_size - 2][i][sim->thread_indx];		// 마지막 waypoint를 preview waypoint로 할당 (X 좌표)
														// 							WP_pv_next[i] = WP[ctrl->WP_size - 1][i][sim->thread_indx];		// 마지막 waypoint를 preview waypoint로 할당 (Y 좌표)
														// 						}
						WP_pv_previous[0] = input->LocalPath[ctrl->WP_size - 2][0];
						WP_pv_previous[1] = input->LocalPath[ctrl->WP_size - 2][1];
						//WP_pv_previous[2] = VehicleVelocity[ctrl->WP_size - 2];
						//WP_pv_previous[3] = StabilityIndex[ctrl->WP_size - 2];
						WP_pv_next[0] = input->LocalPath[ctrl->WP_size - 1][0];
						WP_pv_next[1] = input->LocalPath[ctrl->WP_size - 1][1];
						//WP_pv_next[2] = VehicleVelocity[ctrl->WP_size - 1];
						//WP_pv_next[3] = StabilityIndex[ctrl->WP_size - 1];
						break;
					}

					// 					for (i = 0; i < 4; i++) {
					// 						WP_pv_previous[i] = WP[WP_pv_indx - 1][i][sim->thread_indx];				// 다음 waypoint를 preview waypoint로 할당 (X 좌표)
					// 						WP_pv_next[i] = WP[WP_pv_indx][i][sim->thread_indx];					// 다음 waypoint를 preview waypoint로 할당 (X 좌표)
					// 					}
					WP_pv_previous[0] = input->LocalPath[WP_pv_indx - 1][0];
					WP_pv_previous[1] = input->LocalPath[WP_pv_indx - 1][1];
					//WP_pv_previous[2] = VehicleVelocity[WP_pv_indx - 1];
					//WP_pv_previous[3] = StabilityIndex[WP_pv_indx - 1];
					WP_pv_next[0] = input->LocalPath[WP_pv_indx][0];
					WP_pv_next[1] = input->LocalPath[WP_pv_indx][1];
					//WP_pv_next[2] = VehicleVelocity[WP_pv_indx];
					//WP_pv_next[3] = StabilityIndex[WP_pv_indx];

					d_rest = sqrt((WP_pv_next[0] - WP_pv_previous[0])*(WP_pv_next[0] - WP_pv_previous[0]) + (WP_pv_next[1] - WP_pv_previous[1])*(WP_pv_next[1] - WP_pv_previous[1]));	// 다음 waypoint의 WP_next까지 남은 거리 다시 계산
					L_pv_rest = L_pv_rest - d_rest;		// 초과된 만큼을 계산하여 L_pv_rest에 저장 (초과될 경우 양수, 초과되지 않을 경우 음수를 가진다.)
				}
				L_pv_rest = d_rest + L_pv_rest;			// L_pv_rest가 더이상 초과되지 않는 음수를 가진 경우 d_rest와 더해주어 해당 preview waypoint에서 남은 거리를 다시 계산한다.
				yaw_wp_pv = atan2((WP_pv_next[1] - WP_pv_previous[1]), (WP_pv_next[0] - WP_pv_previous[0]));	// Preview waypoint의 방향 (각도)
				x_pv = WP_pv_previous[0] + L_pv_rest * cos(yaw_wp_pv);	// Preview waypoint의 방향과 남은 거리를 이용하여 preview waypoint 벡터 내에서의 지향점(x_pv)를 계산한다.
				y_pv = WP_pv_previous[1] + L_pv_rest * sin(yaw_wp_pv);	// Preview waypoint의 방향과 남은 거리를 이용하여 preview waypoint 벡터 내에서의 지향점(y_pv)를 계산한다.
			}
			else {		// Preview distance가 현재 waypoint의 범위를 초과되지 않은 경우
				x_pv = x_n + L_pv * cos(yaw_wp);	// 현재 waypoint의 방향과 남은 거리를 이용하여 waypoint 벡터 내에서의 지향점(x_pv)를 계산한다.
				y_pv = y_n + L_pv * sin(yaw_wp);	// 현재 waypoint의 방향과 남은 거리를 이용하여 waypoint 벡터 내에서의 지향점(y_pv)를 계산한다.
			}
		}
		else {			// WP_indx가 WP_indx의 마지막 지점에 도달한 경우
			x_pv = x_n + L_pv * cos(yaw_wp);	// 현재 waypoint, (=마지막 waypoint)에서 waypoint의 방향과 preview distance를 이용하여 지향점 할당 (x_pv)
			y_pv = y_n + L_pv * sin(yaw_wp);	// 현재 waypoint, (=마지막 waypoint)에서 waypoint의 방향과 preview distance를 이용하여 지향점 할당 (y_pv)
		}

		ctrl->yaw_d = atan2((y_pv - y_c), (x_pv - x_c));    // 차량 무게중심으로부터 지향점(preview point)까지의 벡터의 방향을 지향각 명령(desired yaw angle)로 계산한다.
	}

	if (simulation_flag == 1) {		// Equilibrium 상태에서는 최초에 한번 지향 각 명령을 현재 지향각으로 설정함
		if (t_current == 0.0) {
			ctrl->yaw_d = yaw;
		}
		ctrl->e_l = 0;						// save_data()에 사용하기 위한 입력
	}

	////////// Section 2. 속도 및 지향각 명령 계산 ////////// 
	////////// Input: 차량 주행 속도 (v_x), 지향각 (yaw), 지향각속도 (yaw_rate), 종 방향 타이어력(F_x)
	////////// Output: 종 방향 전체 힘 (F_xd_total), 요 모멘트 제어 명령 (M_c)

	// 외란 요 모멘트 추정 관측기
	M_c_true = t_w / 2 * (F_x[1] + F_x[3] + F_x[5] - F_x[0] - F_x[2] - F_x[4]);     // 실제 타이어력으로부터 발생하는 모멘트 계산
	ctrl->yaw_hat = ctrl->yaw_hat + T / I_z * ctrl->M_d_hat + T / I_z * M_c_true + T * p*(yaw_rate - ctrl->yaw_hat);	// yaw rate estimate
	ctrl->M_d_hat = ctrl->M_d_hat + T * eta*(yaw_rate - ctrl->yaw_hat);	// disturbance moment estimate	

																		// 지향각 오차 계산
	if (yaw >= 0) {										// 지향각이 1,2 사분면에 위치한 경우 (양수)
		if (ctrl->yaw_d < (yaw - M_PI)) {					// 지향각 명령이 (지향각-pi)보다 작은 경우
			ctrl->e_psi = ctrl->yaw_d + 2 * M_PI - yaw;
		}
		else {												// 지향각 명령이 (지향각-pi)보다 큰 경우
			ctrl->e_psi = ctrl->yaw_d - yaw;
		}
	}
	else {												// 지향각이 3,4 사분면에 위치한 경우 (음수)
		if (ctrl->yaw_d >(yaw + M_PI)) {					// 지향각 명령이 (지향각+pi)보다 큰 경우
			ctrl->e_psi = (ctrl->yaw_d - 2 * M_PI) - yaw;
		}
		else {												// 지향각 명령이 (지향각+pi)보다 작은 경우
			ctrl->e_psi = ctrl->yaw_d - yaw;
		}
	}

	// Stanley method	
	double e_y_input = K_yp * ctrl->e_psi + atan2(e_y_k*ctrl->e_l, fabs(ctrl->v_x) + vel_offset);

	// 지향 각속도 오차 계산
	de_psi = -yaw_rate;	// de_psi = desired_yaw_rate - yaw_rate (여기서 desired_yaw_rate는 0으로 놓고 계산함)

						// desired velocity
	if (simulation_flag == 1) {	// Equilibrium 상태인 경우
		ctrl->v_di = 0;					// Equilibrium 상태일 때는 차량 속도를 0으로 제어
	}
	else {
		ctrl->v_di = ctrl->v_d_init;
	}

	// 주행 속도 오차 계산
	if (simulation_flag == 1) {
		ctrl->e_v = ctrl->v_di - ctrl->v_x;				// 속도 오차 (Equilibrium)
	}
	else {
		ctrl->e_v = ctrl->v_di - ctrl->v_x;
	}

	ctrl->e_v_sum = ctrl->e_v_sum + ctrl->e_v*T;	// 속도 오차 적분


													// 종방향 힘 및 제어 모멘트 명령
	if (simulation_flag == 1) {
		ctrl->F_xd_total = m_vehicle * (K_vp*ctrl->e_v + K_vi * ctrl->e_v_sum);		// 종 방향 전체 힘 명령
	}
	else {
		ctrl->F_xd_total = m_vehicle * (K_vp*ctrl->e_v + K_vi * ctrl->e_v_sum);
	}
	M_c = I_z * (K_yd*de_psi + e_y_input);					// 요 모멘트 제어 명령

	////////// Section 3. 인-휠 모터 토크 명령 계산 //////////
	////////// Output: 각 휠의 모터 토크 (motoreque[6])

	// 종 방향 타이어력 각 휠에 분배
	F_x_A[0][0] = 2 * (W1 / ((F_z[0] + 1)*(F_z[0] + 1)) + W5 / ((F_z[4] + 1)*(F_z[4] + 1)));
	F_x_A[0][1] = 0;
	F_x_A[0][2] = 2 * W5 / ((F_z[4] + 1)*(F_z[4] + 1));
	F_x_A[0][3] = 0;
	F_x_A[1][0] = 0;
	F_x_A[1][1] = 2 * (W2 / ((F_z[1] + 1)*(F_z[1] + 1)) + W6 / ((F_z[5] + 1)*(F_z[5] + 1)));
	F_x_A[1][2] = 0;
	F_x_A[1][3] = 2 * W6 / ((F_z[5] + 1)*(F_z[5] + 1));
	F_x_A[2][0] = 2 * W5 / ((F_z[4] + 1)*(F_z[4] + 1));
	F_x_A[2][1] = 0;
	F_x_A[2][2] = 2 * (W3 / ((F_z[2] + 1)*(F_z[2] + 1)) + W5 / ((F_z[4] + 1)*(F_z[4] + 1)));
	F_x_A[2][3] = 0;
	F_x_A[3][0] = 0;
	F_x_A[3][1] = 2 * W6 / ((F_z[5] + 1)*(F_z[5] + 1));
	F_x_A[3][2] = 0;
	F_x_A[3][3] = 2 * (W4 / ((F_z[3] + 1)*(F_z[3] + 1)) + W6 / ((F_z[5] + 1)*(F_z[5] + 1)));

	F_x_B[0] = W5 / ((F_z[4] + 1)*(F_z[4] + 1))*ctrl->F_xd_total - 2 * W5 / ((F_z[4] + 1)*(F_z[4] + 1))*M_c / t_w;
	F_x_B[1] = W6 / ((F_z[5] + 1)*(F_z[5] + 1))*ctrl->F_xd_total + 2 * W6 / ((F_z[5] + 1)*(F_z[5] + 1))*M_c / t_w;
	F_x_B[2] = W5 / ((F_z[4] + 1)*(F_z[4] + 1))*ctrl->F_xd_total - 2 * W5 / ((F_z[4] + 1)*(F_z[4] + 1))*M_c / t_w;
	F_x_B[3] = W6 / ((F_z[5] + 1)*(F_z[5] + 1))*ctrl->F_xd_total + 2 * W6 / ((F_z[5] + 1)*(F_z[5] + 1))*M_c / t_w;

	// LU decomposition 방법으로 F_x_1234 = inv(F_x_A)*F_x_B 계산 (2016.11.29 홍효성)
	ludcmp4(F_x_A, 4, indx, 0.0, M_fac);
	lubksb4(M_fac, 4, indx, F_x_B, F_x_1234);

	// 각 휠에서의 종 방향 타이어력 제어 명령
	F_xd[0] = F_x_1234[0];
	F_xd[1] = F_x_1234[1];
	F_xd[2] = F_x_1234[2];
	F_xd[3] = F_x_1234[3];
	F_xd[4] = ctrl->F_xd_total / 2 - M_c / t_w - F_xd[0] - F_xd[2];
	F_xd[5] = ctrl->F_xd_total / 2 + M_c / t_w - F_xd[1] - F_xd[3];

	// 종 방향 타이어력 명령이 한계를 초과할 경우 한계치 만큼만 제어 명령을 가지도록 각 휠에 비례 분배시킨다.
	if (ctrl->F_xd_total > 0 && (F_xd[0] > F_tire_limit || F_xd[1] > F_tire_limit || F_xd[2] > F_tire_limit || F_xd[3] > F_tire_limit || F_xd[4] > F_tire_limit || F_xd[5] > F_tire_limit))
	{
		limit_ratio = F_tire_limit / Find_Max(F_xd, 6);	// 최대 타이어력 제어 명령을 타이어력 한계치 수준에 맞추기 위한 비율 계산
		for (j = 0; j < 6; j++) {
			F_xd[j] = F_xd[j] * limit_ratio;
		}
	}

	// 종방향 타이어력 오차 버퍼에 저장
	for (i = 0; i < 9; i++) {
		for (j = 0; j < 6; j++) {
			ctrl->F_x_error[i][j] = ctrl->F_x_error[i + 1][j];
		}
	}
	for (j = 0; j < 6; j++) {
		ctrl->F_x_error[9][j] = F_xd[j] - F_x[j];
		ctrl->F_x_error_sum[j] = 0;
	}

	for (j = 0; j < 6; j++) {
		for (i = 0; i < 10; i++) {
			ctrl->F_x_error_sum[j] += ctrl->F_x_error[i][j];
		}
	}

	// 인-휠 모터 토크 명령 계산
	for (j = 0; j < 6; j++) {
		ctrl->motor_torque[j] = tire_radius * F_xd[j];		// 모터 토크 명령 = 타이어 반지름 * 종 방향 힘 명령
															//ctrl->motor_torque[j] = tire_radius*(F_xd[j] + 3*(ctrl->F_x_error_sum[j])/10);		// 종방향 타이어 오차 보상

		if (F_z[j] == 0)
			ctrl->motor_torque[j] = 0;						// 수직 타이어력이 0 인 경우 모터 토크도 0으로 설정
	}
}