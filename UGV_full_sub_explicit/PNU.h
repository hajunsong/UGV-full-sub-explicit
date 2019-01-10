#pragma once


/************************************************************************/
/*                              참고                                    */
/************************************************************************/
// 각 라이브러리 별로, 설명, lib 링크, include, namespace 지정 순으로 나열
// #include ""의 주소는 소스코드위치 기준
// #pragma comment의 주소는 프로젝트위치 기준

/************************************************************************/
/*                              std                                     */
/************************************************************************/
//#include <stdio.h>

typedef unsigned int uint;

#include <string>		
// using std::string;

#include <iostream>	
// using std::cout; using std::endl;

#include <vector>		
// using std::vector; 

#include <deque>		
// using std::deque;

class Obj_Map
{

private:
	static const double*const*_map_data;
	static uint _num_x; //포인터로 수정
	static uint _num_y;
	static double _gab;
	static double _x_c;
	static double _y_c;
	static double *_x_data; //갯수가 런타임 결정이기 때문에 배열 사용곤란
	static double *_y_data;

public:
	~Obj_Map();
	static void PNU_set_map(double **map_data, uint rows, uint cols, double gab, double x_c, double y_c);
	static void PNU_fn_map(double x_p, double y_p, double *z);

private:
	static void fn_linspace(double *data, double L, unsigned int num_data, double c);
	static void fn_find_patch_uv(uint* patch_r, uint* patch_c, double* u, double* v, double x_p, double y_p);
	static void fn_MBMt_uv(double(*MBMt)[4], double* u, double* v, double x_p, double y_p);
	static void fn_patch_idxs(int row_idx[4], int col_idx[4], uint patch_r, uint patch_c);
	static void fn_B_mat(double(*B)[4], uint patch_r, uint patch_c);
	static void fn_mat444_multi(double(*C)[4], double(*A)[4], double(*B)[4]);
	static void fn_mat1441_multi(double *A, double(*B)[4], double *C, double *output);
};

class Obj_Nan
{
private:
	static double z_min; //포인터로 수정

public:
	static void fn_nan_interp(double **Map_data, uint n_rows, uint n_cols);

private:
	static double fn_nan_patch(double **Map_data, uint n_rows, uint n_cols, uint i, uint j);
	static int is_nan(double z);
	static void fn_ij_p(int* i_p, int* j_p, uint p, uint k, uint i, uint j);
};

class Obj_Tire
{
private:
	//Obj_Map* _C_Map;

	double _c_t, _k_t, _Iyy, _mus, _mud, _w, _rr, _Ca, _CSLIP, _R_u, _t_s, _slope_rr;
	double _eps, _pi;
	double _c_x, _c_y;
	double _slip_x, _slip_y;

	double _Fx, _Fy, _Fz, _R_e, _R_d, _My; // 출력을 위한 저장변수
	double _F_tire[3], _M_tire[3], _F_global[3], _M_global[3];
	double _slip, _angle;
	double _pen, _road_h; // 디버깅을 위한 임시 변수

	double _Road_h_i[6], _Fric_i[6], _N_vec[3];

	double _P[8]; //타이어 파라미터

public:
	Obj_Tire(); //생성자
	void PNU_Pre_road(double R_c_RF[3], double R_c_RM[3], double R_c_RR[3], double R_c_LF[3], double R_c_LM[3], double R_c_LR[3]);
	void PNU_Tire_force(double R_c[3], double dR_c[3], double u_vec[3], double omega, int index);
	void PNU_get_data(double F[3], double M[3], double *slip, double *angle, double *road_h, double *pen, double *R_d, double *Fx, double *Fy, double *Fz, double *My);
	double PNU_get_unloaded_radius() { return _R_u; };
	void set_Road_h(double road[6]) { memcpy(_Road_h_i, road, sizeof(double) * 6); }
	void set_N_vec(double vec[3]) { memcpy(_N_vec, vec, sizeof(double) * 3); }

private:
	void fn_R_eff(double *R_e, double *a, double R_d);
	void fn_road_tire_A(double road_A[3][3], double tire_A[3][3], double road_z_Vec[3], double tire_y_Vec[3]);
	//void fn_slip_ratio(double *slip, double *angle, double V_x, double omega, double R_e, double V_sy, double Fz);
	void fn_slip_ratio(double *slip_x, double *slip_y, double V_x, double omega, double R_e, double V_sy, double Fz, double a);
	void fn_Fiala_tire(double F[3], double M[3], double Fz, double slip, double alpha, double omega);
	void fn_Brush_tire(double F[3], double M[3], double Fz, double slip_x, double slip_y, double mu, double a, double pen, double omega);
	void fn_step_s(double x, double slope, double *output);
	void fn_cross(double c[3], double a[3], double b[3]);
	void fn_mat3331(double c[3], double a[3][3], double b[3]);
	void fn_mat33T31(double c[3], double a[3][3], double b[3]);
	void fn_sign(double a, double *output);
	void fn_max(double a, double b, double *output);
	void fn_min(double a, double b, double *output);
	void fn_normalize(double a[], uint num);
};