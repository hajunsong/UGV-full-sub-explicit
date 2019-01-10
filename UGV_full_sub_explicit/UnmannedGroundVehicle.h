#pragma once

#include "PNU.h"
#include <iostream>
#include <string>
#include <direct.h>
#include <fstream>
#include <stdio.h>
#include <math.h>

#define subsystems 6
#define TINY 1.0e-20

using namespace std;

class chassis {
public:
	// read chassis
	double r0[3], p0[4], pi0[3], dr0[3], w0[3], dr0p[3], rho0p[3], C00[3][3], m0, J0p[3][3];
	// orientation chassis
	double A0[3][3], E0[3][4], G0[3][4], roll_ang, pitch_ang, yaw_ang;
	// position chassis
	double rho0[3], r0c[3];
	// velocity state chassis
	double r0t[3][3], Y0h[6];
	// cartesian velocity chassis
	double dp0[4], T0[6][6], w0t[3][3], dr0c[3], dr0t[3][3];
	// mass force state chassis
	double J0c[3][3], r0ct[3][3], dr0ct[3][3], M0h[6][6], F0c[3], T0c[3], Q0h[6], Q0h_g[6], L0[6], K0[6][6];
	// acceleration state chassis
	double dY0h[6];
	// cartesian acceleration chassis
	double ddr0[3], dw0[3], dw0t[3][3], ddr0c[3], dT0[6][6], dY0b[6], ddp0[4];
	// local variable
	double dr0cp[3], ddr0cp[3], w0p[3];

	double L0_delay[6], Psi0_y0h[6][6], delta_y0h[6], p0_delay[4], dp0_delay[4], ddp0_delay[4], Pq_delay, Myy[6][6];
	double r0_r0[3][3], dY0ht[3][3], dr0_dr0[3][3], w0_w0[3][3], r0c_r0[3][3], r0ct_dw0_tilde[3][3];
	double rho0_p0[3][4], r0c_p0[3][4], C00_J0p_C00t[3][3], A0_CJACT_dw0_p0[3][4], A0t_dw0_p0[3][4], J0c_dw0_p0[3][4];
	double M0h_dY0h_r0[6][3], M0h_dY0h_p0[6][4], M0_y0h[6][6], F0ct[3][3], dr0ct_w0_tilde[3][3], Q0h_g_r0[6][3];
	double A0_CJACT_w0_p0[3][4], A0t_w0_p0[3][4], J0c_w0_p0[3][4], dr0c_p0[3][4], Q0h_g_p0[6][4], Q0_y0h[6][6];
	double dr0c_dr0[3][3], Q0h_g_dr0[6][3], rho0t[3][3], dr0c_w0[3][3], J0c_w0_tilde[3][3], Q0h_g_w0[6][3], Q0_dy0h[6][6], delta_y0[6];
	double Q0h_r0[6][3], L0_r0[6][3], Q0h_p0[6][4], L0_p0[6][4], Q0h_g_dro[6][3], Q0h_dr0[6][3], L0_dr0[6][3], Q0h_w0[6][3];
	double L0_w0[6][3], Py_y0h[6][6], Myy_y0h[6][6], Py_dy0h[6][6], J0_y0h[6][6], J0_dy0h[6][6], delta_dy0h[6], dY0h_delay[6], Y0h_delay[6];
};

class suspension {
public:
	int id;
	// read suspension
	double q1, dq1, rho1p[3], C11[3][3], m1, J1p[3][3], s01p[3], s12p[3], C01[3][3], C12[3][3], s0sp[3], s1sp[3];
	// orientation suspension
	double A1[3][3], H1[3], A01pp[3][3];
	// position suspension
	double r1[3], s12[3], rw[3], rho1[3], r1c[3], s01[3];
	// velocity state suspension
	double r1t[3][3], B1[6], Y1h[6];
	// cartesian velocity suspension
	double T1[6][6], Y1b[6], dr1[3], w1[3], w1t[3][3], dr1c[3], drwc[3];
	// mass force state suspension
	double J1c[3][3], r1ct[3][3], dr1ct[3][3], M1h[6][6], F1c[3], s12t[3][3], rho1t[3][3], T1c[3], Qh1[6], F_tire[3], M_tire[3], Q1h_g[6];
	double kt, R_z, R_u, point_h, r_road[3], r_p2a[3], tF_tire[3][3];
	// CNU TSDA
	double L_free, s0s[3], s1s[3], d01[3], poly_factor[8], L_spring, defo, T_spring, T_damper, dL_spring, f, C, s0st[3][3], s1st[3][3], Q0h_TSDA[6], Q1h_TSDA[6], f_L;
	double Q0h_TSDA_temp[6], Q1h_TSDA_temp[6], dL_spring_temp[3];
	// velocity coupling
	double dr1t[3][3], D1[6], dH1[3], D1_temp[6];
	// effective mass force
	double Myq[6], Pq, inv_Mqq, Mhc[6][6], Qhc[6], L1[6], K1[6][6], Myy[6][6], Mqq, Py[6], K1_D1[6];
	// acceleration state suspension
	double dY1h[6], ddq1;
	// cartesian acceleration suspension
	double ddr1[3], dw1[3], dw1t[3][3], ddr1c[3], dT1[6][6], dY1b[6];
	// wheel & tire
	double theta_wh, w_wh, dw_wh, T_in;
	// Tire 계산 후 결과 저장 변수 - slip ratio, slip angle, Tire longitudinal force(Local), Tire lateral force(Local), Tire vertical force(Local), deformed radius, penetration, height of road, Moment y axis(타이어 회전 축, local)
	double slip, angle, Fx, Fy, Fz, R_d, pen, road_h, My;

	double Py_delay[6], EffJaco1[6][6], EffJaco2[6], q1_delay, dq1_delay, ddq1_delay, Psi1_y0h[6], delta_q, Psi1_q1, Pq_delay;
	double r1_r0[3][3], r1c_r0[3][3], H1t[3][3], r1ct_H1_tilde[3][3], rho1_p0[3][4], r1_p0[3][4], r1c_p0[3][4], r1t_H1_tilde[3][3];
	double C11_J1p_C11t[3][3], A1_CJACT_dw0_p0[3][4], A1t_dw0_p0[3][4], J1c_dw0_p0[3][4], A1_CJACT_H1_p0[3][4], A1t_H1_p0[3][4], H1_p0[3][4];
	double M1h_dY0h_r0[6][3], r1ct_dw0_tilde[3][3], M1h_dY0h_p0[6][4], K1_B1_ddq1_r0[6][3], Ht1[3][3], K1_B1_ddq1_p0[6][4], B1_r0[6][3];
	double B1t_K1_dY0h_r0[3], M1h_dY0h[6][6], B1t_K1_dY0h_p0[4], B1t_K1_B1_ddq1_r0[3], B1_p0[6][4], B1t_K1_B1_ddq1_p0[4];
	double Mi_r0[3], Mi_p0[4], Mi_y0[6], Mi_y0h[6], Miy0[6], s01_p0[3][4];
	double A01pp_q1[3][3], A1_q1[3][3], r1c_q1[3], J1c_q1[3][3], M1h_dY0h_q1[6], K1_dY0h_q1[6], K1_B1_ddq1_q1[6], M0_qi[6], Mi_qi[6];
	double F1ct[3][3], dr1ct_w1_tilde[3][3], d01t[3][3], dH1t[3][3], dr1t_H1_r1t_dH1_tilde[3][3], r1ct_dH1_tilde[3][3], rw_r0[3][3];
	double r_p2a_r0[3][3], r_p2a_t[3][3], F_tire_r0[3][3], M_tire_r0[3][3], Q0h_TSDA_r0[6][3], Q1h_g_r0[6][3], Q1h_TSDA_r0[6][3], Q1h_r0[6][3];
	double K1_D1_r0[6][3], L1_r0[6][3], s1s_p0[3][4], s0s_p0[3][4], d01_p0[3][4], L_spring_p0[4], defo_p0[4], f_spring_p0[4], dr1_p0[3][4], w1_p0[3][4];
	double w1t_s1s_p0[3][4], dL_spring_p0[4], f_damper_p0[4], f_p0[4], f_L_p0[4], J1c_w1_tilde[3][3], s12_p0[3][4], rw_p0[3][4];
	double r_p2a_p0[3][4], F_tire_p0[3][4], M_tire_p0[3][4], dr1c_p0[3][4], A1_CJACT_w1_p0[3][4], A1t_w1_p0[3][4], J1c_w1_p0[3][4];
	double A1_CJACT_dH1_p0[3][4], A1t_dH1_p0[3][4], dH1_p0[3][4], J1c_dH1_p0[3][4], Q0h_TSDA_p0[6][4], Q1h_g_p0[6][4];
	double Q1h_TSDA_p0[6][4], Q1h_p0[6][4], K1_D1_p0[6][4], L1_p0[6][4], Qi_r0[3], Qi_p0[4], Qi_y0[6], Qi_y0h[6], d01_q1[3], L_spring_q1, defo_q1, dL_spring_q1;
	double f_spring_q1, f_damper_q1, f_q1, f_L_q1, Q0h_TSDA_q1[6], Q1h_TSDA_q1[6], dr1c_q1[3], rw_q1[3], r_p2a_q1[3], F_tire_q1[3], M_tire_q1[3], Q1h_g_q1[6];
	double Q0h_q1[6], Q1h_q1[6], K1_D1_q1[6], L1_q1[6], Q0_qi[6], Qi_qi[6], dr1_dr0[3][3], dr1c_dr0[3][3], Q1h_g_dr0[6][3];
	double K1_D1_dr0[6][3], Q1h_dr0[6][3], L1_dr0[6][3], w1_w0[3][3], s01t[3][3], dr1_w0[3][3], dr1c_w0[3][3];
	double dH1_w0[3][3], Q1h_w0[6][3], L1_w0[6][3], Qi_dr0[3], Qi_w0[3], K1_D1_w0[6][3], Qi_dy0[6];
	double Qi_dy0h[6], w1_dq1[3], dr1c_dq1[3], dL_spring_dq1, f_damper_dq1, f_dq1, Q0h_TSDA_dq1[6], Q1h_TSDA_dq1[6], Q1h_g_dq1[6];
	double Q0h_dq1[6], Q1h_dq1[6], K1_D1_dq1[6], L1_dq1[6], Q0_dqi[6], Qi_dqi[6], Psi0_q1[6], M1h_dY0h_y0h[6][6], M1h_dY0h_y0[6][6], K1_B1_ddq1_y0[6][6];
	double K1_B1_ddq1_y0h[6][6], Q0h_TSDA_y0[6][6], Q0h_TSDA_y0h[6][6], J1c_H1_p0[3][4];
	double Myy_y0h[6][6], Myq_y0h[6][6], Py_y0h[6][6], Py_dy0h[6][6], Pq_y0h[6], Mqy_y0h[6], Mqq_y0h[6], Pq_dy0h[6], Py_q1[6];
	double Myy_q1[6], Py_dq1[6], Pq_q1, Mqy_q1, Mqq_q1, Pq_dq1, J1_y0h[6], J1_dy0h[6], J0_q1[6], J1_q1, J1_dq1, Myq_q1[6], J0_dq1[6];
	double subsystem_factor[6], effective_factor1[6][6], effective_factor2[6], delta_dq, Q1h_g_w0[6][3];
};

class controller {
public:
	// controller variable
	int WP_indx, WP_size, WP_indx_update_flag;	// parallel simulation 코드와 optimal velocity control 코드에서 사용	
	double yaw_d, e_v_sum, M_d_hat, yaw_hat;			// read control에서 사용				   
	double v_di, v_d, v_x, v_d_init;					// LP stability metric, save data, equilibrium, parallel process 등에서 사용
	double e_l, e_psi;						// LP stability metric에서 사용 (lateral position error, 지향각 오차)
	double F_xd_total, e_v;						// save data에서 사용
	double F_x_error[10][6], F_x_error_sum[6];		// F_xi(desired - actual)
	double motor_torque[6];	//output		
};

class input_data {
public:
	// Map Info
	int MapInfo_Row, MapInfo_Col, MapInfo_Robot_Row, MapInfo_Robot_Col;
	double MapInfo_Resolution_Row, MapInfo_Resolution_Col, MapInfo_Resolution_Down, MapInfo_ReferenceNorth, MapInfo_ReferenceEast, **ElevationData;
	// Navigation Info
	double Velocity_North, Velocity_East, Velocity_Down, Velocity, Attitude_Yaw;
	// Local Path Info
	int WaypointSize;
	double **LocalPath;
};

class UnmannedGroundVehicle {
public:
	UnmannedGroundVehicle();
	~UnmannedGroundVehicle();

	void init();
	void equilibrium();
	void run();
private:
	input_data *input;

	chassis *chs;
	suspension *sus;
	Obj_Tire *tire;
	controller *ctrl;

	const double M_PI = 3.14159265358979323846;

	const int RF = 0;
	const int RM = 1;
	const int RR = 2;
	const int LF = 3;
	const int LM = 4;
	const int LR = 5;

	const double CH_in[35] = { 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 4362, 2046.8, 0, 0, 0, 7113.2, 0, 0, 0, 7474.2 };
	const double RF_in[54] = { 0, 0, 0.586234043, 0, 9.32E-02, -6.48E-12, 0.865112866, -0.501577241, 1, 3.04E-12, -7.68E-12, -5.12E-12, -0.501577241, -0.865112866, 376, 31.25706362, 0, 0, 0, 25.98627239, 0, 0, 0, 20.83704122, 1.13, -0.755, -0.108, 0.65, 0, 0.219, 1, 0, 0, 0, -5.10E-12, -1, 0, 1, -5.10E-12, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1.276, -0.736, 0.658, 0.281, 0.133, -1.90E-02 };
	const double RM_in[54] = { 0, 0, 0.585382979, 0, 9.32E-02, 1.53E-11, -0.868782601, -0.495193692, -1, -1.33E-11, -7.58E-12, 0, 0.495193692, -0.868782601, 376, 31.21662106, 0, 0, 0, 25.94143616, 0, 0, 0, 20.8414349, 0.83, -0.755, -0.108, 0.648, 0, 0.219, -1, 1.02E-11, 0, 5.21E-23, 5.10E-12, -1, -1.02E-11, -1, -5.10E-12, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0.683, -0.736, 0.658, 0.281, -0.133, -1.90E-02 };
	const double RR_in[54] = { 0, 0, 0.585382979, 0, 0.0932, 0.0000000000153, -0.868782601, -0.495193692, -1, -0.0000000000133, -0.00000000000758, 0, 0.495193692, -0.868782601, 376, 31.21662106, 0, 0, 0, 25.94143616, 0, 0, 0, 20.8414349, -0.77, -0.755, -0.108, 0.648, 0, 0.219, -1, 0.0000000000102, 0, 5.21E-23, 0.0000000000051, -1, -0.0000000000102, -1, -0.0000000000051, 1, 0, 0, 0, 1, 0, 0, 0, 1, -0.917, -0.736, 0.658, 0.281, -0.133, -0.019};
	const double LF_in[54] = { 0, 0, 0.585864863, 0, -0.0932, -0.0000000000051, 0.866708022, 0.498815802, 1, 0.00000000000442, 0.00000000000255, 0, 0.498815802, -0.866708022, 376, 31.23950966, 0, 0, 0, 25.96680966, 0, 0, 0, 20.83895001, 1.13, 0.755, -0.108, 0.65, 0, -0.219, 1, 0, 0, 0, -0.0000000000051, -1, 0, 1, -0.0000000000051, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1.276, 0.736, 0.658, 0.281, 0.133, 0.019};
	const double LM_in[54] = { 0, 0, 0.585752158, 0, -0.0932, -0.00000000000375, -0.867194003, 0.497970441, -1, 0.00000000000578, 0.00000000000254, -0.00000000000508, -0.497970441, -0.867194003, 376, 31.23415389, 0, 0, 0, 25.96087203, 0, 0, 0, 20.83953186, 0.83, 0.755, -0.108, 0.648, 0, -0.219, -1, 0.0000000000102, 0, 5.21E-23, 0.0000000000051, -1, -0.0000000000102, -1, -0.0000000000051, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0.683, 0.736, 0.658, 0.281, -0.133, 0.019};
	const double LR_in[54] = { 0, 0, 0.585382979, 0, -0.0932, -0.00000000000376, -0.868782601, 0.495193692, -1, 0.00000000000577, 0.00000000000253, -0.00000000000505, -0.495193692, -0.868782601, 376, 31.21662106, 0, 0, 0, 25.94143616, 0, 0, 0, 20.8414349, -0.77, 0.755, -0.108, 0.648, 0, -0.219, -1, 0.0000000000102, 0, 5.21E-23, 0.0000000000051, -1, -0.0000000000102, -1, -0.0000000000051, 1, 0, 0, 0, 1, 0, 0, 0, 1, -0.917, 0.736, 0.658, 0.281, -0.133, 0.019};

	double start_time, end_time, h, g, t_current, t_next;
	double Y[31], Yp[31], Y_next[31], Y_equil[13];
	string file_name, dir;
	FILE *fp;
	int intcount;
	double AW[31][2], AW1[31][2];
	double M[6][6], Q[6];
	double q[6], q_dot[6], q_ddot[6];
	double alpha, beta, gamma, ddq_q, dq_q;
	double dYb_dYh[6][6], yb_yh[6][6], p0_pi0[4][3];
	double q_delay[6], q_dot_delay[6], q_ddot_delay[6], delta_q[6];

	double heading;
	int wp_size, simulation_flag;

	void read_data();
		void read_system();
		void read_chassis();
			void read_RF(suspension *sus);
			void read_RM(suspension *sus);
			void read_RR(suspension *sus);
			void read_LF(suspension *sus);
			void read_LM(suspension *sus);
			void read_LR(suspension *sus);
		void read_control();

	void define_Y_vector();

	void Y2qdq();

	void var2qdqddq();
	void save_delay();
	void qdqddq2var();
	void chassis_jacobian();
	void suspension_jacobian(int i);
		void jaco_A0_vector_p0(double v[3], double p[4], double A0_v_p[3][4]);
		void jaco_A0t_vector_p0(double v[3], double p[4], double A0t_v_p[3][4]);

	void analysis();
		void orientation_chassis();
		void position_chassis();
		void velocity_state_chassis();
		void cartesian_velocity_chassis();
		void mass_force_state_chassis();
		void orientation_suspension(suspension *sus);
		void position_suspension(suspension *sus);
		void velocity_state_suspension(suspension *sus);
		void cartesian_velocity_suspension(suspension *sus);
		void pre_road(suspension *sus);
		void mass_force_state_suspension(suspension *sus);
		void effective_mass_force(suspension *sus);
			void TSDA(suspension *sus);
			void TIRE(suspension *sus);
		void velocity_coupling(suspension *sus);
		void acceleration_state_chassis();
		void cartesian_acceleration_chassis();
		void acceleration_state_suspension(suspension *sus);
		void cartesian_acceleration_suspension(suspension *sus);

		void LP_control();
		void wheel_spin_dyn();

	void dqddq2Yp();

	void save_data();

	void absh3(double step_size, const int n);
	void tilde(double a[3], double b[3][3]);
	void mat33T31(double a[3][3], double b[3], double c[3]);
	void mat3333(double a[3][3], double b[3][3], double c[3][3]);
	void mat3331(double a[3][3], double b[3], double c[3]);
	void mat333333(double a[3][3], double b[3][3], double c[3][3], double d[3][3]);
	void mat34T31(double a[3][4], double b[3], double c[4]);
	void mat3333T(double a[3][3], double b[3][3], double c[3][3]);
	void mat333331(double a[3][3], double b[3][3], double c[3], double d[3]);
	void mat6661(double a[6][6], double b[6], double c[6]);
	void mat61T61(double a[6], double b[6], double *c);
	void mat6161T(double a[6], double b[6], double c[6][6]);
	void mat333333T(double a[3][3], double b[3][3], double c[3][3], double d[3][3]);
	void mat3334(double a[3][3], double b[3][4], double c[3][4]);
	void mat33T34(double a[3][3], double b[3][4], double c[3][4]);
	void ludcmp6(double a[6][6], int n, int indx[6], double d, double a_fac[6][6]);
	void lubksb6(double a_fac[6][6], int n, int indx[6], double b[6], double x[6]);

	double Find_Max(double Array[], int length);
	double Find_min(double data, double min_val);
	double fsign(double data);
	void ludcmp4(double a[4][4], int n, int indx[4], double d, double fac[4][4]);
	void lubksb4(double a_fac[4][4], int n, int indx[4], double b[4], double x[4]);
};

#ifndef CSVPARSER_H
#define CSVPARSER_H

#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

	typedef struct CsvRow {
		char **fields_;
		int numOfFields_;
	} CsvRow;

	typedef struct CsvParser {
		char *filePath_;
		char delimiter_;
		int firstLineIsHeader_;
		char *errMsg_;
		CsvRow *header_;
		FILE *fileHandler_;
		int fromString_;
		char *csvString_;
		int csvStringIter_;
	} CsvParser;


	// Public
	CsvParser *CsvParser_new(const char *filePath, const char *delimiter, int firstLineIsHeader);
	CsvParser *CsvParser_new_from_string(const char *csvString, const char *delimiter, int firstLineIsHeader);
	void CsvParser_destroy(CsvParser *csvParser);
	void CsvParser_destroy_row(CsvRow *csvRow);
	const CsvRow *CsvParser_getHeader(CsvParser *csvParser);
	CsvRow *CsvParser_getRow(CsvParser *csvParser);
	int CsvParser_getNumFields(const CsvRow *csvRow);
	const char **CsvParser_getFields(const CsvRow *csvRow);
	const char* CsvParser_getErrorMessage(CsvParser *csvParser);

	// Private
	CsvRow *_CsvParser_getRow(CsvParser *csvParser);
	int _CsvParser_delimiterIsAccepted(const char *delimiter);
	void _CsvParser_setErrorMessage(CsvParser *csvParser, const char *errorMessage);

#ifdef __cplusplus
}
#endif

#endif