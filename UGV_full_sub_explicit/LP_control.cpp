#include "UnmannedGroundVehicle.h"

void UnmannedGroundVehicle::LP_control() {
	/*
	LP_control() : ���� ���� �����
	�ۼ���: ȫȿ��
	Date: 2017.02.17

	input variables
	v_di: desired velocity
	WP : Waypoint data
	Yp : current derivative of state
	step_size: integration step_size

	output variables
	motor_torque: �� �ٿ� ���޵Ǵ� ���� ��ũ ���

	*/

	////////// Section 0. ���� �Ķ���� �� ���� ���� �ʱ�ȭ //////////

	////////// �Է� �޾ƾ��� ���� �� �Ķ���� //////////
	double m_vehicle = 6762;		// vehicle mass [kg]
	double t_w = 1.948;             // ��, ���� �� �� �Ÿ�(m) = tread	(���� 2016.11.30 ȫȿ��)	
	double I_z = 13201;             // total moment of inertia of z - axis(kgm ^ 2)
	double tire_radius = tire->PNU_get_unloaded_radius();	// Ÿ�̾� ������
	double torque_limit = 3416;		//[Nm], ����: 17.08 ����
									//double torque_limit = 10000;
									////////// ���� ���� //////////
	int i, j;						// �ݺ����� ����ϱ� ���� ����
	int indx[4] = { 0 };			// lusolve4 (LU decomponent) �Լ����� ����ϱ� ���� ����
	double M_fac[4][4] = { 0 };		// lusolve4 (LU decomponent) �Լ����� ����ϱ� ���� ����
	int WP_pv_indx;						// preview point ����� ���� waypoint �ε���
	double x_c, y_c;				// �۷ι� ��ǥ���� �� ���� ���� �߽��� ��ġ
	double yaw, yaw_rate;			//�� ����, �� ���ӵ�
	double L = 1.2;                   // Preview distance �Ÿ� ����(m)
	double F_z[6], F_x[6];			// ���� ���� Ÿ�̾��, �� ���� Ÿ�̾��
	double W1, W2, W3, W4, W5, W6;	// Ÿ�̾�� �й迡 ���Ǵ� weight factor	
	double F_x_A[4][4], F_x_B[4], F_x_1234[4], F_xd[6];	// Ÿ�̾�� �й迡 ���Ǵ� ���
	double K_vp, K_vi, K_yp, K_yd;	// ���� ����
	double M_c, M_c_true;			// �� ���Ʈ ���� ��� �� ���� Ÿ�̾�¿��� �߻��Ǵ� �� ���Ʈ
	double T;						// step time
	double l, eta, p;				// �ܶ� �� ���Ʈ �����⿡ ���Ǵ� ����
	double F_tire_limit;				// �� ���� Ÿ�̾�� ��� limit, 
	double limit_ratio;				//���� ��ũ ����� �Ѱ� ���� �ʰ��� ��� ��ũ�� ��� ����Ͽ� ���� ���� ����
	double WP_previous[4], WP_next[4];			// WP_previous: ������ ��ġ�ϰ� �ִ� ���� waypoint, WP_next: WP_previous ������ waypoint
	double WP_pv_previous[4], WP_pv_next[4];	// preview point ����� ���� ���� waypoint �� ���� waypoint
	double d_wp;						// �� waypoint ������ �Ÿ� (WP_previous ���� WP_next ����)
	double yaw_wp;					// ���� �Ҽӵ� waypoint�� ����(����)
	double u_wp;					// ���� �Ҽӵ� waypoint ������ ������ ��ġ�� ��������� ��Ÿ���� ���� ���� (0~1���� ����, u_wp > 1 �� ��� ������ ���� waypoint�� �����ƴٴ� �ǹ�)
	double x_n, y_n;				// ���� �Ҽӵ� waypoint ���� ������ ���� �����߽ɿ� ���� ���� ������ ������
	double L_pv; // preview distance 
	double d_rest, L_pv_rest;	// preview distance�� ���� ���� ������������ �Ÿ��� �ش� waypoint �������� Ŭ ��� �ʰ��� ��ŭ�� �Ÿ� (preview waypoint�� update �� �� ���)
	double x_pv, y_pv, yaw_wp_pv; // preview distance�� ���� ������ ��ǥ(x_pv, y_pv) �� ���� �����߽����κ����� ���� ��
	double de_psi;	// ���Ⱒ�ӵ� ���� (desired yaw_rate - current yaw_rate)

					// �ӵ� �� ���Ⱒ ���� ����
	K_vp = 10;
	K_vi = 0;
	K_yd = 10;
	K_yp = 30;
	double e_y_k = 40;	// K gain for stanley method
	double vel_offset = 20; // for stanley method

	////////// ��� ���� ��� ����ϱ� ���� ���� ���� ���� �Է� //////////

	yaw = chs->yaw_ang;     // ������ ���Ⱒ (rad)
	yaw_rate = chs->w0[2];  // ���Ⱒ�ӵ� (rad/s)	
	x_c = chs->r0c[0];		// �۷ι� ��ǥ������ ���� �����߽��� x ��ǥ
	y_c = chs->r0c[1];		// �۷ι� ��ǥ������ ���� �����߽��� y ��ǥ
	ctrl->v_x = chs->dr0cp[0];		// ������ ���� x ���� �ӵ�(LOCAL)

	// ���� ���� Ÿ�̾�� (Ÿ�̾� ���� �����κ��� �Է�)
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

	// �� ���� Ÿ�̾�� (Ÿ�̾� ���� �����κ��� �Է�)
	F_x[0] = sus[LF].Fx;		// LF (Left, Front)
	F_x[1] = sus[RF].Fx;		// FR (Right, Front)
	F_x[2] = sus[LM].Fx;		// LM (Left, Middle)
	F_x[3] = sus[RM].Fx;		// RM (Right, Middle)
	F_x[4] = sus[LR].Fx;		// LR (Left, Right)
	F_x[5] = sus[RR].Fx;		// RR (Right, Rear)

								// Ÿ�̾�� �й迡 ���Ǵ� weight factor	����
	W1 = 1; W2 = 1; W3 = 1; W4 = 1; W5 = 1; W6 = 1;

	// �ܶ� �� ���Ʈ ������ ����
	T = h;					// step time (������ �𵨰� ����ȭ)
	l = 10 * I_z;				// observer gain
	eta = 25 * I_z;				// observer gain
	p = l / I_z;				// observer gain

	// �� ���� Ÿ�̾�� ����
	F_tire_limit = torque_limit / tire_radius;

	// waypoint ���� ���� ���� ���� ���� �ӵ� �Է� (optimal_velocity_profile�� ����ϱ� ���� �ʱ�ȭ ����)
	//WP[0][2][sim->thread_indx] = ctrl->v_x;	

	//VehicleVelocity[0] = ctrl->v_x;

	////////// Section 1. ���� �� ��� ////////// 
	////////// Input: ���� ���� ��ġ (x_c, y_c), Waypoint data (WP)
	////////// Output: ���Ⱒ ��� (yaw_d)

	if (simulation_flag == 2) {										// ���� ���� (RTT) ����� ��쿡�� ��� (Equilibrium ��忡���� ��� ����)
		// 		for (i = 0; i < 4; i++) {
		// 			WP_previous[i] = WP[ctrl->WP_indx - 1][i][sim->thread_indx];			// ���� ������ �Ҽӵ� ��ǥ�� WP_previous�� waypoint�� X, Y ��ǥ�� ���� �ӵ� ��� �׸��� stability indx = 0 �Ҵ�
		// 			WP_next[i] = WP[ctrl->WP_indx][i][sim->thread_indx];				// ���� ��ǥ�� ���� ��ǥ�� WP_next�� waypoint�� X, Y ��ǥ�� ���� �ӵ� ��� �׸��� stability indx = 0 �Ҵ�
		// 		}
		WP_previous[0] = input->LocalPath[ctrl->WP_indx - 1][0];
		WP_previous[1] = input->LocalPath[ctrl->WP_indx - 1][1];
		//WP_previous[2] = VehicleVelocity[ctrl->WP_indx - 1];
		//WP_previous[3] = StabilityIndex[ctrl->WP_indx - 1];
		WP_next[0] = input->LocalPath[ctrl->WP_indx][0];
		WP_next[1] = input->LocalPath[ctrl->WP_indx][1];
		//WP_next[2] = VehicleVelocity[ctrl->WP_indx];
		//WP_next[3] = StabilityIndex[ctrl->WP_indx];


		d_wp = sqrt((WP_next[0] - WP_previous[0])*(WP_next[0] - WP_previous[0]) + (WP_next[1] - WP_previous[1])*(WP_next[1] - WP_previous[1]));		// WP_previous�� WP_next ������ �Ÿ� ���
		yaw_wp = atan2((WP_next[1] - WP_previous[1]), (WP_next[0] - WP_previous[0]));										// waypoint�� direction(rad) ���
		u_wp = ((x_c - WP_previous[0])*(WP_next[0] - WP_previous[0]) + (y_c - WP_previous[1])*(WP_next[1] - WP_previous[1])) / (d_wp*d_wp);	// WP_previous�� WP_next ���̿��� ������ ��� ��ġ�� ������ ��� (0 ~ 1 ����)

		while (u_wp > 1)													// u_wp > 1 : ������ ��ġ�� WP_next ������ �ʰ��� ��� -> ���ο� waypoint �Ҵ�
		{
			//WP[ctrl->WP_indx][2][sim->thread_indx] = ctrl->v_x;			// �ش� WP_indx �տ����� ���� ���� �ӵ� �Է�
			//VehicleVelocity[ctrl->WP_indx] = ctrl->v_x;

			if (ctrl->WP_indx >= ctrl->WP_size - 1) {						// WP_indx�� ������ ���� �����ϸ� �� �̻� WP_indx�� ������Ʈ ���� ����
				break;
			}

			ctrl->WP_indx++;												// WP_previous�� WP_next ��ǥ�� ���� ������Ʈ�ϱ� ���� WP_indx�� +1 ��Ų��.
			ctrl->WP_indx_update_flag = 1;									// WP_indx�� ������Ʈ �� ��� LP_stability_metric()�� ������� �� stability indx �׸��� 1�� �ʱ�ȭ �����ش�.

																			// waypoint ���� �Ҵ�
																			// 			for (i = 0; i < 4; i++) {
																			// 				WP_previous[i] = WP[ctrl->WP_indx - 1][i][sim->thread_indx];		// ���� ������ �Ҽӵ� ��ǥ�� WP_previous�� waypoint�� X, Y ��ǥ�� ���� �ӵ� ��� �׸��� stability indx = 0 �Ҵ�
																			// 				WP_next[i] = WP[ctrl->WP_indx][i][sim->thread_indx];			// ���� ������ �Ҽӵ� ��ǥ�� WP_next�� waypoint�� X, Y ��ǥ�� ���� �ӵ� ��� �׸��� stability indx = 0 �Ҵ�
																			// 			}
			WP_previous[0] = input->LocalPath[ctrl->WP_indx - 1][0];
			WP_previous[1] = input->LocalPath[ctrl->WP_indx - 1][1];
			//WP_previous[2] = VehicleVelocity[ctrl->WP_indx - 1];
			//WP_previous[3] = StabilityIndex[ctrl->WP_indx - 1];
			WP_next[0] = input->LocalPath[ctrl->WP_indx][0];
			WP_next[1] = input->LocalPath[ctrl->WP_indx][1];
			//WP_next[2] = VehicleVelocity[ctrl->WP_indx];
			//WP_next[3] = StabilityIndex[ctrl->WP_indx];

			d_wp = sqrt((WP_next[0] - WP_previous[0])*(WP_next[0] - WP_previous[0]) + (WP_next[1] - WP_previous[1])*(WP_next[1] - WP_previous[1]));		// WP_previous�� WP_next ������ �Ÿ� ���
			yaw_wp = atan2((WP_next[1] - WP_previous[1]), (WP_next[0] - WP_previous[0]));										// waypoint�� direction(rad) ���
			u_wp = ((x_c - WP_previous[0])*(WP_next[0] - WP_previous[0]) + (y_c - WP_previous[1])*(WP_next[1] - WP_previous[1])) / (d_wp*d_wp);	// WP_previous�� WP_next ���̿��� ������ ��� ��ġ�� ������ ��� (0 ~ 1 ����)
		}

		x_n = WP_previous[0] + u_wp * (WP_next[0] - WP_previous[0]);		// ���� �Ҽӵ� waypoint ���� ������ ���� �����߽ɿ� ���� ���� ������ ������ X ��ǥ
		y_n = WP_previous[1] + u_wp * (WP_next[1] - WP_previous[1]);		// ���� �Ҽӵ� waypoint ���� ������ ���� �����߽ɿ� ���� ���� ������ ������ Y ��ǥ

		// Ⱦ ���� ��ġ ���� ��� (lateral position error)
		//if (y_n > y_c) {							// ���� LPE�� �׻� ����� �������� ��ο� ���Ͽ� ���� Y ��ǥ�� ��� ��ġ�� ���� ��ȣ�� �ٲ�� ���̵��� �ӽ� ��ġ��
		//	ctrl->e_l = sqrt((x_n - x_c)*(x_n - x_c) + (y_n - y_c)*(y_n - y_c));		// e_l : Ⱦ ���� ��ġ ����
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
		if (fabs(ctrl->v_x) > L) {		// Preview distance�� ���� ���� �ӵ��� ����ϵ��� ������
			L_pv = fabs(ctrl->v_x)*L;
		}
		else {
			L_pv = L;					// ���� ���� �ӵ��� 1 m/s ���� ���� ���� L_pv = 1 (m) �� ������. ���⼭ L = 1 �̴�.
		}

		if (ctrl->WP_indx < ctrl->WP_size) {	// WP_indx�� WP_indx�� ������ ������ �������� �ʾ��� ���
			WP_pv_indx = ctrl->WP_indx;			// Preview waypoint�� indx�� ���� waypoint�� indx�� ����ȭ
												//d_rest = sqrt((WP[WP_pv_indx][0][sim->thread_indx] - x_n)*(WP[WP_pv_indx][0][sim->thread_indx] - x_n) + (WP[WP_pv_indx][1][sim->thread_indx] - y_n)*(WP[WP_pv_indx][1][sim->thread_indx] - y_n));	// ���� waypoint�� ������ WP_next���� ���� �Ÿ� (distance_rest)
			d_rest = sqrt((input->LocalPath[WP_pv_indx][0] - x_n) * (input->LocalPath[WP_pv_indx][0] - x_n) + (input->LocalPath[WP_pv_indx][1] - y_n)*(input->LocalPath[WP_pv_indx][1] - y_n));

			if (L_pv > d_rest) {			// Preview distance�� ���� waypoint�� ������ �ʰ��� ���
				L_pv_rest = L_pv - d_rest;	// �ʰ��� ��ŭ�� ����Ͽ� L_pv_rest�� ����
				while (L_pv_rest > 0) {		// �ʰ��� �Ÿ��� ����̸�					
					WP_pv_indx = WP_pv_indx + 1;		// Preview waypoint�� indx�� +1 ������Ʈ ��Ŵ (Preview waypoint ������Ʈ�� ����)
					if (WP_pv_indx >= ctrl->WP_size) {	// ���� Preview waypoint�� indx�� ��ü waypoint indx ������ �ʰ��� ��� ������ waypoint�� preview point�� �Ҵ�
														// 						for (i = 0; i < 4; i++) {
														// 							WP_pv_previous[i] = WP[ctrl->WP_size - 2][i][sim->thread_indx];		// ������ waypoint�� preview waypoint�� �Ҵ� (X ��ǥ)
														// 							WP_pv_next[i] = WP[ctrl->WP_size - 1][i][sim->thread_indx];		// ������ waypoint�� preview waypoint�� �Ҵ� (Y ��ǥ)
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
					// 						WP_pv_previous[i] = WP[WP_pv_indx - 1][i][sim->thread_indx];				// ���� waypoint�� preview waypoint�� �Ҵ� (X ��ǥ)
					// 						WP_pv_next[i] = WP[WP_pv_indx][i][sim->thread_indx];					// ���� waypoint�� preview waypoint�� �Ҵ� (X ��ǥ)
					// 					}
					WP_pv_previous[0] = input->LocalPath[WP_pv_indx - 1][0];
					WP_pv_previous[1] = input->LocalPath[WP_pv_indx - 1][1];
					//WP_pv_previous[2] = VehicleVelocity[WP_pv_indx - 1];
					//WP_pv_previous[3] = StabilityIndex[WP_pv_indx - 1];
					WP_pv_next[0] = input->LocalPath[WP_pv_indx][0];
					WP_pv_next[1] = input->LocalPath[WP_pv_indx][1];
					//WP_pv_next[2] = VehicleVelocity[WP_pv_indx];
					//WP_pv_next[3] = StabilityIndex[WP_pv_indx];

					d_rest = sqrt((WP_pv_next[0] - WP_pv_previous[0])*(WP_pv_next[0] - WP_pv_previous[0]) + (WP_pv_next[1] - WP_pv_previous[1])*(WP_pv_next[1] - WP_pv_previous[1]));	// ���� waypoint�� WP_next���� ���� �Ÿ� �ٽ� ���
					L_pv_rest = L_pv_rest - d_rest;		// �ʰ��� ��ŭ�� ����Ͽ� L_pv_rest�� ���� (�ʰ��� ��� ���, �ʰ����� ���� ��� ������ ������.)
				}
				L_pv_rest = d_rest + L_pv_rest;			// L_pv_rest�� ���̻� �ʰ����� �ʴ� ������ ���� ��� d_rest�� �����־� �ش� preview waypoint���� ���� �Ÿ��� �ٽ� ����Ѵ�.
				yaw_wp_pv = atan2((WP_pv_next[1] - WP_pv_previous[1]), (WP_pv_next[0] - WP_pv_previous[0]));	// Preview waypoint�� ���� (����)
				x_pv = WP_pv_previous[0] + L_pv_rest * cos(yaw_wp_pv);	// Preview waypoint�� ����� ���� �Ÿ��� �̿��Ͽ� preview waypoint ���� �������� ������(x_pv)�� ����Ѵ�.
				y_pv = WP_pv_previous[1] + L_pv_rest * sin(yaw_wp_pv);	// Preview waypoint�� ����� ���� �Ÿ��� �̿��Ͽ� preview waypoint ���� �������� ������(y_pv)�� ����Ѵ�.
			}
			else {		// Preview distance�� ���� waypoint�� ������ �ʰ����� ���� ���
				x_pv = x_n + L_pv * cos(yaw_wp);	// ���� waypoint�� ����� ���� �Ÿ��� �̿��Ͽ� waypoint ���� �������� ������(x_pv)�� ����Ѵ�.
				y_pv = y_n + L_pv * sin(yaw_wp);	// ���� waypoint�� ����� ���� �Ÿ��� �̿��Ͽ� waypoint ���� �������� ������(y_pv)�� ����Ѵ�.
			}
		}
		else {			// WP_indx�� WP_indx�� ������ ������ ������ ���
			x_pv = x_n + L_pv * cos(yaw_wp);	// ���� waypoint, (=������ waypoint)���� waypoint�� ����� preview distance�� �̿��Ͽ� ������ �Ҵ� (x_pv)
			y_pv = y_n + L_pv * sin(yaw_wp);	// ���� waypoint, (=������ waypoint)���� waypoint�� ����� preview distance�� �̿��Ͽ� ������ �Ҵ� (y_pv)
		}

		ctrl->yaw_d = atan2((y_pv - y_c), (x_pv - x_c));    // ���� �����߽����κ��� ������(preview point)������ ������ ������ ���Ⱒ ���(desired yaw angle)�� ����Ѵ�.
	}

	if (simulation_flag == 1) {		// Equilibrium ���¿����� ���ʿ� �ѹ� ���� �� ����� ���� ���Ⱒ���� ������
		if (t_current == 0.0) {
			ctrl->yaw_d = yaw;
		}
		ctrl->e_l = 0;						// save_data()�� ����ϱ� ���� �Է�
	}

	////////// Section 2. �ӵ� �� ���Ⱒ ��� ��� ////////// 
	////////// Input: ���� ���� �ӵ� (v_x), ���Ⱒ (yaw), ���Ⱒ�ӵ� (yaw_rate), �� ���� Ÿ�̾��(F_x)
	////////// Output: �� ���� ��ü �� (F_xd_total), �� ���Ʈ ���� ��� (M_c)

	// �ܶ� �� ���Ʈ ���� ������
	M_c_true = t_w / 2 * (F_x[1] + F_x[3] + F_x[5] - F_x[0] - F_x[2] - F_x[4]);     // ���� Ÿ�̾�����κ��� �߻��ϴ� ���Ʈ ���
	ctrl->yaw_hat = ctrl->yaw_hat + T / I_z * ctrl->M_d_hat + T / I_z * M_c_true + T * p*(yaw_rate - ctrl->yaw_hat);	// yaw rate estimate
	ctrl->M_d_hat = ctrl->M_d_hat + T * eta*(yaw_rate - ctrl->yaw_hat);	// disturbance moment estimate	

																		// ���Ⱒ ���� ���
	if (yaw >= 0) {										// ���Ⱒ�� 1,2 ��и鿡 ��ġ�� ��� (���)
		if (ctrl->yaw_d < (yaw - M_PI)) {					// ���Ⱒ ����� (���Ⱒ-pi)���� ���� ���
			ctrl->e_psi = ctrl->yaw_d + 2 * M_PI - yaw;
		}
		else {												// ���Ⱒ ����� (���Ⱒ-pi)���� ū ���
			ctrl->e_psi = ctrl->yaw_d - yaw;
		}
	}
	else {												// ���Ⱒ�� 3,4 ��и鿡 ��ġ�� ��� (����)
		if (ctrl->yaw_d >(yaw + M_PI)) {					// ���Ⱒ ����� (���Ⱒ+pi)���� ū ���
			ctrl->e_psi = (ctrl->yaw_d - 2 * M_PI) - yaw;
		}
		else {												// ���Ⱒ ����� (���Ⱒ+pi)���� ���� ���
			ctrl->e_psi = ctrl->yaw_d - yaw;
		}
	}

	// Stanley method	
	double e_y_input = K_yp * ctrl->e_psi + atan2(e_y_k*ctrl->e_l, fabs(ctrl->v_x) + vel_offset);

	// ���� ���ӵ� ���� ���
	de_psi = -yaw_rate;	// de_psi = desired_yaw_rate - yaw_rate (���⼭ desired_yaw_rate�� 0���� ���� �����)

						// desired velocity
	if (simulation_flag == 1) {	// Equilibrium ������ ���
		ctrl->v_di = 0;					// Equilibrium ������ ���� ���� �ӵ��� 0���� ����
	}
	else {
		ctrl->v_di = ctrl->v_d_init;
	}

	// ���� �ӵ� ���� ���
	if (simulation_flag == 1) {
		ctrl->e_v = ctrl->v_di - ctrl->v_x;				// �ӵ� ���� (Equilibrium)
	}
	else {
		ctrl->e_v = ctrl->v_di - ctrl->v_x;
	}

	ctrl->e_v_sum = ctrl->e_v_sum + ctrl->e_v*T;	// �ӵ� ���� ����


													// ������ �� �� ���� ���Ʈ ���
	if (simulation_flag == 1) {
		ctrl->F_xd_total = m_vehicle * (K_vp*ctrl->e_v + K_vi * ctrl->e_v_sum);		// �� ���� ��ü �� ���
	}
	else {
		ctrl->F_xd_total = m_vehicle * (K_vp*ctrl->e_v + K_vi * ctrl->e_v_sum);
	}
	M_c = I_z * (K_yd*de_psi + e_y_input);					// �� ���Ʈ ���� ���

	////////// Section 3. ��-�� ���� ��ũ ��� ��� //////////
	////////// Output: �� ���� ���� ��ũ (motoreque[6])

	// �� ���� Ÿ�̾�� �� �ٿ� �й�
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

	// LU decomposition ������� F_x_1234 = inv(F_x_A)*F_x_B ��� (2016.11.29 ȫȿ��)
	ludcmp4(F_x_A, 4, indx, 0.0, M_fac);
	lubksb4(M_fac, 4, indx, F_x_B, F_x_1234);

	// �� �ٿ����� �� ���� Ÿ�̾�� ���� ���
	F_xd[0] = F_x_1234[0];
	F_xd[1] = F_x_1234[1];
	F_xd[2] = F_x_1234[2];
	F_xd[3] = F_x_1234[3];
	F_xd[4] = ctrl->F_xd_total / 2 - M_c / t_w - F_xd[0] - F_xd[2];
	F_xd[5] = ctrl->F_xd_total / 2 + M_c / t_w - F_xd[1] - F_xd[3];

	// �� ���� Ÿ�̾�� ����� �Ѱ踦 �ʰ��� ��� �Ѱ�ġ ��ŭ�� ���� ����� �������� �� �ٿ� ��� �й��Ų��.
	if (ctrl->F_xd_total > 0 && (F_xd[0] > F_tire_limit || F_xd[1] > F_tire_limit || F_xd[2] > F_tire_limit || F_xd[3] > F_tire_limit || F_xd[4] > F_tire_limit || F_xd[5] > F_tire_limit))
	{
		limit_ratio = F_tire_limit / Find_Max(F_xd, 6);	// �ִ� Ÿ�̾�� ���� ����� Ÿ�̾�� �Ѱ�ġ ���ؿ� ���߱� ���� ���� ���
		for (j = 0; j < 6; j++) {
			F_xd[j] = F_xd[j] * limit_ratio;
		}
	}

	// ������ Ÿ�̾�� ���� ���ۿ� ����
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

	// ��-�� ���� ��ũ ��� ���
	for (j = 0; j < 6; j++) {
		ctrl->motor_torque[j] = tire_radius * F_xd[j];		// ���� ��ũ ��� = Ÿ�̾� ������ * �� ���� �� ���
															//ctrl->motor_torque[j] = tire_radius*(F_xd[j] + 3*(ctrl->F_x_error_sum[j])/10);		// ������ Ÿ�̾� ���� ����

		if (F_z[j] == 0)
			ctrl->motor_torque[j] = 0;						// ���� Ÿ�̾���� 0 �� ��� ���� ��ũ�� 0���� ����
	}
}