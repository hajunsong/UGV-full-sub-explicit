#include "PNU.h"

#pragma warning(disable:4996)

using namespace std;

uint	Obj_Map::_num_x; //�����ͷ� ����
uint	Obj_Map::_num_y;
double	Obj_Map::_gab;
double	Obj_Map::_x_c;
double	Obj_Map::_y_c;
double	*Obj_Map::_x_data = NULL; //������ ��Ÿ�� �����̱� ������ �迭 �����
double	*Obj_Map::_y_data = NULL;
const double*const*	Obj_Map::_map_data = NULL; //���������� ���ó����� ���� ����

Obj_Map::~Obj_Map()
{
	delete[] _x_data;
	_x_data = NULL;

	delete[] _y_data;
	_y_data = NULL;
}

void Obj_Map::PNU_set_map(double **map_data, uint rows, uint cols, double gab, double x_c, double y_c)
{
	double length_x, length_y;
	_map_data = map_data;
	//_map_data[0][0]=0.0;
	_num_x = cols;
	_num_y = rows;
	_gab = gab;
	_x_c = x_c;
	_y_c = y_c;

	length_x = (double)(_num_x - 1)*gab;
	length_y = (double)(_num_y - 1)*gab;

	delete[] _x_data;
	delete[] _y_data;

	_x_data = new double[_num_x];
	_y_data = new double[_num_y];

	fn_linspace(_x_data, length_x, _num_x, _x_c);
	fn_linspace(_y_data, length_y, _num_y, _y_c);
}

void Obj_Map::PNU_fn_map(double x_p, double y_p, double *z)
{
	double MBMt[4][4], Ut[4], V[4];
	double u, v;
	uint i;
	if (_map_data == NULL || _x_data == NULL || _y_data == NULL) {
		printf("\n ����� �ʱ�ȭ ���� ����! ���̰��� 0���� �ӽ����!! \n");
		*z = 0;
	}
	else {
		fn_MBMt_uv(MBMt, &u, &v, x_p, y_p);

		for (i = 0; i < 4; ++i) {
			Ut[i] = pow(u, (int)(i));
			V[i] = pow(v, (int)(i));
		}
		fn_mat1441_multi(V, MBMt, Ut, z);
	}
}

void Obj_Map::fn_linspace(double *data, double L, uint num_data, double c)
{
	double gab = L / (double)(num_data - 1);
	uint i;
	for (i = 0; i < num_data; ++i) {
		data[i] = gab * (double)(i)-c;
	}
}

void Obj_Map::fn_find_patch_uv(uint* patch_r, uint* patch_c, double* u, double* v, double x_p, double y_p)
{
	double gab_th = 0.00000001; //magic, ��谪 ó���� ����
	double x_p_m, y_p_m, x_len, y_len;
	uint sw_error;

	x_p_m = x_p;
	y_p_m = y_p;

	sw_error = 0;
	if (x_p_m < _x_data[0]) {
		x_p_m = _x_data[0];
		sw_error = 1;
	} //x�� ����
	if (x_p_m >= _x_data[_num_x - 1]) {
		x_p_m = _x_data[_num_x - 1] - gab_th;
		sw_error = 1;
	} //x�� ����, ���� ������ >���� >=���� ����
	if (y_p_m < _y_data[0]) {
		y_p_m = _y_data[0];
		sw_error = 1;
	} //y�� ����
	if (y_p_m >= _y_data[_num_y - 1]) {
		y_p_m = _y_data[_num_y - 1] - gab_th;
		sw_error = 1;
	} //y�� ����, ���� ������ >���� >=���� ����

	  // ����ó�� �޼��� ǥ�ÿ��� Ȯ��
	if (sw_error) {
		//printf("xy_position�� data������ �Ѿ \n");
	}

	////////// patch ���
	x_len = x_p_m + _x_c; //���������δ� ���� ��ġ�� row, column�� ù ��°�̱� ����
	y_len = y_p_m + _y_c;

	*patch_r = (uint)floor(y_len / _gab); // ���� ������ (y_len/_gab+gab_th)���� (y_len/_gab) ���� ����
	*patch_c = (uint)floor(x_len / _gab);

	////////// uv���
	*u = fmod(x_len, _gab) / _gab;
	*v = fmod(y_len, _gab) / _gab;
}

void Obj_Map::fn_MBMt_uv(double(*MBMt)[4], double* u, double* v, double x_p, double y_p)
{
	double M[4][4] = { { 1, 0, 0, 0 },{ 0, 0, 1, 0 },{ -3, 3, -2, -1 },{ 2, -2, 1, 1 } };
	double B[4][4], MB[4][4], Mt[4][4];
	uint patch_r, patch_c;
	uint i, j;
	fn_find_patch_uv(&patch_r, &patch_c, u, v, x_p, y_p);
	fn_B_mat(B, patch_r, patch_c);

	for (j = 0; j < 4; ++j) {
		for (i = 0; i < 4; ++i) {
			Mt[i][j] = M[j][i];
		}
	}

	fn_mat444_multi(MB, M, B);
	fn_mat444_multi(MBMt, MB, Mt);
}

void Obj_Map::fn_patch_idxs(int row_idx[4], int col_idx[4], uint patch_r, uint patch_c)
{
	int idx_set[4] = { -1, 0, 1, 2 };
	uint i;
	for (i = 0; i < 4; ++i) {
		col_idx[i] = idx_set[i] + (int)(patch_c);
		row_idx[i] = idx_set[i] + (int)(patch_r);

		if (col_idx[i] >= (int)(_num_x)) //uword�ε� col_idx�� ivec���� ǥ���ϱⰡ ������...
			col_idx[i] = (int)(_num_x - 1);
		else if (col_idx[i] < 0)
			col_idx[i] = 0;

		if (row_idx[i] >= (int)(_num_y))
			row_idx[i] = (int)(_num_y - 1);
		else if (row_idx[i] < 0)
			row_idx[i] = 0;
	}
}

void Obj_Map::fn_B_mat(double(*B)[4], uint patch_r, uint patch_c)
{
	int row_idx[4];
	int col_idx[4];
	double tmp;
	double R[2][2], Rv[2][2], Ru[2][2], Ruv[2][2];
	uint i, j;

	fn_patch_idxs(row_idx, col_idx, patch_r, patch_c);

	R[0][0] = _map_data[row_idx[1]][col_idx[1]];
	R[0][1] = _map_data[row_idx[1]][col_idx[2]];
	R[1][0] = _map_data[row_idx[2]][col_idx[1]];
	R[1][1] = _map_data[row_idx[2]][col_idx[2]];

	tmp = 1 / (2.0*_gab);
	Rv[0][0] = (_map_data[row_idx[2]][col_idx[1]] - _map_data[row_idx[0]][col_idx[1]])*tmp;
	Rv[1][0] = (_map_data[row_idx[3]][col_idx[1]] - _map_data[row_idx[0]][col_idx[1]])*tmp;
	Rv[0][1] = (_map_data[row_idx[2]][col_idx[2]] - _map_data[row_idx[0]][col_idx[2]])*tmp;
	Rv[1][1] = (_map_data[row_idx[3]][col_idx[2]] - _map_data[row_idx[0]][col_idx[2]])*tmp;

	Ru[0][0] = (_map_data[row_idx[1]][col_idx[2]] - _map_data[row_idx[1]][col_idx[0]])*tmp;
	Ru[1][0] = (_map_data[row_idx[2]][col_idx[2]] - _map_data[row_idx[2]][col_idx[0]])*tmp;
	Ru[0][1] = (_map_data[row_idx[1]][col_idx[3]] - _map_data[row_idx[1]][col_idx[1]])*tmp;
	Ru[1][1] = (_map_data[row_idx[2]][col_idx[3]] - _map_data[row_idx[2]][col_idx[1]])*tmp;

	Ruv[0][0] = (Ru[0][0] + Rv[0][0])*0.5;
	Ruv[0][1] = (Ru[0][1] + Rv[0][1])*0.5;
	Ruv[1][0] = (Ru[1][0] + Rv[1][0])*0.5;
	Ruv[1][1] = (Ru[1][1] + Rv[1][1])*0.5;

	for (j = 0; j < 2; ++j) {
		for (i = 0; i < 2; ++i) {
			B[i][j] = R[i][j];
			B[i][j + 2] = Ru[i][j];
			B[i + 2][j] = Rv[i][j];
			B[i + 2][j + 2] = Ruv[i][j];
		}
	}
}

void Obj_Map::fn_mat444_multi(double(*C)[4], double(*A)[4], double(*B)[4])
{
	uint i, j, k;
	for (j = 0; j < 4; ++j) {
		for (i = 0; i < 4; ++i) {
			C[i][j] = 0.0;
			for (k = 0; k < 4; ++k) {
				C[i][j] = C[i][j] + A[i][k] * B[k][j];
			}
		}
	}
}

void Obj_Map::fn_mat1441_multi(double *A, double(*B)[4], double *C, double *output)
{
	double sol = C[0] * (A[0] * B[0][0] + A[1] * B[1][0] + A[2] * B[2][0] + A[3] * B[3][0]) + C[1] * (A[0] * B[0][1] + A[1] * B[1][1] + A[2] * B[2][1] + A[3] * B[3][1]) + C[2] * (A[0] * B[0][2] + A[1] * B[1][2] + A[2] * B[2][2] + A[3] * B[3][2]) + C[3] * (A[0] * B[0][3] + A[1] * B[1][3] + A[2] * B[2][3] + A[3] * B[3][3]);
	*output = sol;
}

double Obj_Nan::z_min = -100.0; // nan �������� �ּ� ����

void Obj_Nan::fn_nan_interp(double **Map_data, uint n_rows, uint n_cols)
{
	uint i = 0, j = 0;
	for (i = 0; i < n_rows; ++i) {
		for (j = 0; j < n_cols; ++j) {

			fn_nan_patch(Map_data, n_rows, n_cols, i, j);

		}
	}
}


double Obj_Nan::fn_nan_patch(double **Map_data, uint n_rows, uint n_cols, uint i, uint j)
{
	//const volatile uint num_p=4; //C89 ������ const �ٿ��� �迭�ʱ�ȭ �Ұ�
	enum array_size { num_p = 4 }; //Ž�� ������ ��, 4 �Ǵ� 8�� ����  // 4�� �밢������, 8�� �밢�� + ��������
								   //array ������ ������ ���� enum�� ���, define ��� ���� ����
	double z, z_p;
	uint k, p;
	int s;
	int i_p, j_p;
	double pr, num, den;
	double k_end, z_end;

	uint sw_set[num_p] = { 0 }; // ��谪 ���� ����, 0 : nan, 1 : ������ ����, 2 : ��� ����(nan��ü��)
	double z_set[num_p] = { 0 };  // ��谪 ���޽� ���̰�
	uint k_set[num_p] = { 1 };  // ��԰� ���޽� �ε����Ÿ�

	z = Map_data[i][j];
	if (is_nan(z) != 1) {
		return z;
	}

	// ��谪 Ž�� ����
	i_p = 0;
	j_p = 0;
	for (p = 0; p < num_p; ++p) {
		if (sw_set[p] != 0) { continue; }

		k = 0;
		while (1) {
			k++;
			fn_ij_p(&i_p, &j_p, p, k, i, j);
			if (i_p < 0 || j_p < 0 || i_p >= (int)(n_rows) || j_p >= (int)(n_cols)) {
				sw_set[p] = 2;
				break;
			}
			else {
				z_p = Map_data[i_p][j_p];
				if (is_nan(z_p)) {
					sw_set[p] = 0;
				}
				else {
					sw_set[p] = 1;
					z_set[p] = z_p;
					k_set[p] = k;
					break;
				}
			}
		}
	}

	// Nan �Ÿ����� ���
	num = 0.0;
	den = 0.0;
	s = 0;
	for (p = 0; p < num_p; p++) {
		if (sw_set[p] == 1) {
			s++;
			pr = 1.0 / k_set[p];
			num = num + z_set[p] * pr;
			den = den + pr;
		}
	}

	if (s != 0) {		//s�� 0�̸� ������ ��谪�� ���⶧��
		z = num / den;
		Map_data[i][j] = z;
	}


	// Ž�� ��� ����
	for (p = 0; p < num_p; ++p) {
		if (sw_set[p] == 2) { continue; }

		k_end = k_set[p];
		z_end = z_set[p];
		for (k = 0; k < k_end; ++k) { //�߿� : matlab�� k<k_end-1 ��, ������ �����ϱ�
			fn_ij_p(&i_p, &j_p, p, k, i, j);
			z_p = (z*(k_end - k) + z_end * k) / k_end;
			Map_data[i_p][j_p] = z_p;
		}
	}

	return z;
}

int Obj_Nan::is_nan(double z)
{
	if (z < z_min) {
		return 1;
	}
	else {
		return 0;
	}
}

void Obj_Nan::fn_ij_p(int* i_p, int* j_p, uint p, uint k, uint i, uint j)
{
	switch (p) {
	case 0: *i_p = i + k;    *j_p = j + k; break;
	case 1: *i_p = i + k;    *j_p = j - k; break;
	case 2: *i_p = i - k;    *j_p = j - k; break;
	case 3: *i_p = i - k;    *j_p = j + k; break;
	case 4: *i_p = i + k;    *j_p = j;   break;
	case 5: *i_p = i - k;    *j_p = j;   break;
	case 6: *i_p = i;      *j_p = j + k; break;
	case 7: *i_p = i;      *j_p = j - k; break;
	}
}

Obj_Tire::Obj_Tire()
{
	_c_t = 0;
	_k_t = 300000;// 490000;
	_Iyy = 21;                               // moment of inertia
	_w = 0.38;                               // patch width
	_R_u = 0.548;                            // Ÿ�̾� ������
	_t_s = 0.2;
	//_slope_rr = 1;                           // �������� ����� ���� 0���� omega�� �ִ����, ���б�� ����

	_eps = 2.22044604925031e-16;
	_pi = 3.141592653589793;

	_P[0] = 1.5260080e+01;
	_P[1] = 1.4351226e+01;
	_P[2] = -1.7859839e+00;
	_P[3] = -6.7357190e-02;
	_P[4] = -1.7717619e+00;
	_P[5] = -2.9680650e+00;
	_P[6] = 7.2179859e-01;
	_P[7] = 2.6179968e+00;

	_c_x = exp(_P[0]);
	_c_y = exp(_P[1]);
}

void Obj_Tire::PNU_Pre_road(double R_c_RF[3], double R_c_RM[3], double R_c_RR[3], double R_c_LF[3], double R_c_LM[3], double R_c_LR[3]) {
	Obj_Map::PNU_fn_map(R_c_RF[0], R_c_RF[1], &_Road_h_i[0]);
	Obj_Map::PNU_fn_map(R_c_RM[0], R_c_RM[1], &_Road_h_i[1]);
	Obj_Map::PNU_fn_map(R_c_RR[0], R_c_RR[1], &_Road_h_i[2]);
	Obj_Map::PNU_fn_map(R_c_LF[0], R_c_LF[1], &_Road_h_i[3]);
	Obj_Map::PNU_fn_map(R_c_LM[0], R_c_LM[1], &_Road_h_i[4]);
	Obj_Map::PNU_fn_map(R_c_LR[0], R_c_LR[1], &_Road_h_i[5]);

	double P_RF[3] = { R_c_RF[0], R_c_RF[1], _Road_h_i[0] };
	double P_RR[3] = { R_c_RR[0], R_c_RR[1], _Road_h_i[2] };
	double P_LF[3] = { R_c_LF[0], R_c_LF[1], _Road_h_i[3] };
	double P_LR[3] = { R_c_LR[0], R_c_LR[1], _Road_h_i[5] };

	double T_vec1[3], T_vec2[3];
	for (int i = 0; i < 3; i++) {
		T_vec1[i] = P_RF[i] - P_LR[i];
		T_vec2[i] = P_LF[i] - P_RR[i];
	}

	fn_cross(_N_vec, T_vec1, T_vec2);

	fn_normalize(_N_vec, 3);
}

void Obj_Tire::PNU_Tire_force(double R_c[3], double dR_c[3], double u_vec[3], double omega, int id)
{
	// ������ ���
	double R_x = R_c[0];
	double R_y = R_c[1];
	//Obj_Map::PNU_fn_map(R_x, R_y, &_road_h);
	//road_h = -0.63; // PNU_fn_map(R_x,R_y);	//������ ����

	//double _N_vec[3] = { 0,0,1 };	// ���� ���� ����
	double R_z = R_c[2];
	double dR_z = dR_c[2];
	double point_h;
	point_h = R_z - _R_u;	//tire�� point follow��
	fn_max(_Road_h_i[id] - point_h, 0.0, &_pen);	//tire penetration ħ����
													//fn_max(_road_h - point_h, 0.0, &_pen);		//tire penetration ħ����

	double Fz;
	if (_pen > 0)
		Fz = _k_t * _pen - (_c_t * dR_z)*(1);
	else if (_pen < 0)
		Fz = _k_t * _pen - (_c_t * dR_z)*(-1);
	else
		Fz = _k_t * _pen - (_c_t * dR_z)*(0);

	// effective radius ���
	double R_e, a;
	fn_max(_R_u - _pen, 0.0, &_R_d);
	fn_R_eff(&R_e, &a, _R_d);

	// ��ǥ�� ����

	//double N_vec[3] = { 0, 0, 1 };
	double A_road[3][3], A_tire[3][3];
	fn_road_tire_A(A_road, A_tire, _N_vec, u_vec);

	// Ÿ�̾� �����ӵ��� global->road(��ġ) ��ǥ��ȯ

	double V_x, V_sy;
	double B_dR_c[3];
	fn_mat33T31(B_dR_c, A_road, dR_c); // A_tire���� A_road�� ����, �̰� �´°� ����
	V_x = B_dR_c[0];
	V_sy = B_dR_c[1];

	// slip_ratio, slip_angle ���
	//fn_slip_ratio(&_slip, &_angle, V_x, omega, R_e, V_sy, Fz);
	fn_slip_ratio(&_slip_x, &_slip_y, V_x, omega, R_e, V_sy, Fz, a);

	// road(��ġ)���� Ÿ�̾� �� ���

	double F_road[3], M_road[3];
	double vec_p2c[3], temp1[3];
	//fn_Fiala_tire(F_road, M_road, Fz, _slip, _angle, omega); //contact patch���� iso

	double mu = 0.8;
	fn_Brush_tire(F_road, M_road, Fz, _slip_x, _slip_y, mu, a, _pen, omega);

	// Ÿ�̾� ���� road->tire ��ǥ��ȯ
	//vec_p2c[0] = 0; //tire�� point center���� point follow�������� vector, ��ǥ�� ��ȯ�� ���
	//vec_p2c[1] = 0;
	//vec_p2c[2] = -_R_d;

	//vec_p2c[0] = -_N_vec[0]; //tire�� point center���� point follow�������� vector, ��ǥ�� ��ȯ�� ���
	//vec_p2c[1] = -_N_vec[1];
	//vec_p2c[2] = -_N_vec[2] * _R_d;

	vec_p2c[0] = -_N_vec[0] * _R_d; //tire�� point center���� point follow�������� vector, ��ǥ�� ��ȯ�� ���
	vec_p2c[1] = -_N_vec[1] * _R_d;
	vec_p2c[2] = -_N_vec[2] * _R_d;

	//_F_road : Contact patch���� �߻��ϴ� road local force
	//_F_global : Wheel center���� �߻��ϴ� global force
	//_F_tire : Wheel center���� �߻��ϴ� tire local force

	fn_mat3331(_F_global, A_road, F_road);
	fn_mat3331(_M_global, A_road, M_road);

	fn_cross(temp1, vec_p2c, _F_global);
	for (uint i = 0; i < 3; i++) {
		_M_global[i] += temp1[i];
	}
	//fn_mat3331(_M_tire, A_road, M_road);
	fn_mat33T31(_F_tire, A_tire, _F_global);
	fn_mat33T31(_M_tire, A_tire, _M_global);

	_Fx = _F_tire[0];
	_Fy = _F_tire[1];
	_Fz = _F_tire[2];

	_My = _M_tire[1];
}

void Obj_Tire::PNU_get_data(double F[3], double M[3], double *slip, double *angle, double *road_h, double *pen, double *R_d, double *Fx, double *Fy, double *Fz, double *My)
{
	memcpy(F, _F_global, sizeof(double) * 3); // 	F = _F_tire;
	memcpy(M, _M_global, sizeof(double) * 3); // 	M = _M_tire;
	*slip = _slip_x;
	*angle = _slip_y;
	*road_h = _road_h;
	*pen = _pen;
	*R_d = _R_d;
	*Fx = _Fx;
	*Fy = _Fy;
	*Fz = _Fz;
	*My = _My;
}

void Obj_Tire::fn_Brush_tire(double F[3], double M[3], double Fz, double slip_x, double slip_y, double mu, double a, double pen, double omega)
{

	double s_x = fabs(slip_x);
	double s_y = fabs(slip_y);

	const double b = _w;

	a = a + _eps;

	double k_x = _c_x / (2 * b);
	double k_y = _c_y / (2 * b);

	double u_s_x = mu * (exp(_P[2]) + exp(pen*_P[3]));
	double	u_s_y = mu * (exp(_P[4]) + exp(pen*_P[5]));

	double u_d_x = u_s_x * (1 - s_x / (1 + exp(_P[6])));
	double u_d_y = u_s_y * (1 - s_y / (1 + exp(_P[7])));

	// Combined adhesion force

	double e = 3 * Fz / (4 * a*a*b);

	double S_s_x = u_s_x / k_x * e + _eps;
	double S_s_y = u_s_y / k_y * e + _eps;
	double S_s;
	fn_min(sqrt(pow(s_x / S_s_x, 2) + pow(s_y / S_s_y, 2)), 1.0, &S_s);
	double X_s;
	fn_max(2 * a*(1 - S_s), 0.0, &X_s);

	double F_ax = b * k_x*s_x*X_s*X_s;
	double F_ay = b * k_y*s_y*X_s*X_s;
	double M_az = (b*k_y*s_y*X_s*X_s*(3 * a - 2 * X_s)) / 3;

	// Combined s sliding force
	double den = sqrt(pow(u_d_x*s_x, 2) + pow(u_d_y*s_y, 2) + _eps);
	u_d_x = u_d_x * (u_d_x*s_x / den);
	u_d_y = u_d_y * (u_d_y*s_y / den);
	double F_sz = (b*e*(a + X_s)*pow(2 * a - X_s, 2)) / (3 * a);

	double F_sx = u_d_x * F_sz;
	double F_sy = u_d_y * F_sz;
	double M_sz = -u_d_y * (b*e*X_s*X_s*pow(2 * a - X_s, 2)) / (4 * a);

	// Sum. of adhesion and sliding forces
	// ISO ���� ����

	double sign_x, sign_y;
	fn_sign(slip_x, &sign_x);
	fn_sign(slip_y, &sign_y);


	double Fx = sign_x * (F_ax + F_sx);
	double Fy = -sign_y * (F_ay + F_sy);
	double Mz = -sign_y * (M_az + M_sz);

	// Output
	F[0] = Fx;
	F[1] = Fy;
	F[2] = Fz;
	M[0] = 0;
	M[1] = 0;
	M[2] = Mz;
}

void Obj_Tire::fn_R_eff(double *R_e, double *a, double R_d) {

	double e = acos(R_d / _R_u);
	*a = _R_u * sin(e);
	*R_e = *a / (e + _eps);
}

void Obj_Tire::fn_road_tire_A(double road_A[3][3], double tire_A[3][3], double road_z_Vec[3], double tire_y_Vec[3]) {

	// tire_Ay�� road_Az�� �����ϸ� ����
	double tire_Ax[3], tire_Ay[3], tire_Az[3];
	double road_Ax[3], road_Ay[3], road_Az[3];
	int i;

	memcpy(road_Az, road_z_Vec, sizeof(double) * 3);
	memcpy(tire_Ay, tire_y_Vec, sizeof(double) * 3);

	// ���� ���͸� �� �Է����شٸ� �ʿ����
	//fn_normalize(road_Az,3);
	//fn_normalize(tire_Ay,3); 

	// ����
	fn_cross(road_Ax, tire_Ay, road_Az);
	fn_cross(road_Ay, road_Az, road_Ax);

	memcpy(tire_Ax, road_Ax, sizeof(double) * 3);

	fn_cross(tire_Az, tire_Ax, tire_Ay);

	// output
	for (i = 0; i < 3; i++) {
		road_A[i][0] = road_Ax[i]; //road�� ��ǥ��
		road_A[i][1] = road_Ay[i];
		road_A[i][2] = road_Az[i];
		tire_A[i][0] = tire_Ax[i]; //tire�� ��ǥ��
		tire_A[i][1] = tire_Ay[i];
		tire_A[i][2] = tire_Az[i];
	}
}

//void Obj_Tire::fn_slip_ratio(double *slip, double *angle, double V_x, double omega, double R_e, double V_sy, double Fz) {
//	double speed_m_x, speed_x, slip_x, speed_m_y, speed_y, slip_y, temp1, temp2;
//
//	speed_m_x = _CSLIP * R_e * R_e * _t_s / (_Iyy);  // ���� �����Ŀ��� Euler method ���� unstable ������ �ӵ�
//	fn_max(fabs(V_x), speed_m_x, &speed_x);
//	slip_x = (R_e*omega - V_x) / speed_x;
//
//	speed_m_y = _Ca*9.81*_t_s / (2 * Fz + _eps);
//	fn_max(fabs(V_x), speed_m_y, &speed_y);
//	slip_y = V_sy / speed_y;
//	fn_max(slip_y, -1.0, &temp1);
//	fn_min(temp1, 1.0, &slip_y);
//
//	fn_max(slip_x, -1.0, &temp2);
//	fn_min(temp2, 1.0, slip);        // combined slip�� �ǹ̸� ����ؼ� - 1, +1�� ����
//	*angle = atan(slip_y); // atan(1) = pi / 4 = 45deg �̱� ������ 45���� �ִ�
//}

void Obj_Tire::fn_slip_ratio(double *slip_x, double *slip_y, double V_x, double omega, double R_e, double V_sy, double Fz, double a) {
	double speed_m_x, speed_x, speed_m_y, speed_y, temp1, temp2;

	_CSLIP = 2 * a*a * _c_x; // N / rad : cornering stiffness
	_Ca = 2 * a*a * _c_y;    // N / m : longitudinal stiffness

	speed_m_x = _CSLIP * R_e * R_e * _t_s / (2 * _Iyy);  // ���� �����Ŀ��� Euler method ���� unstable ������ �ӵ�
	fn_max(fabs(V_x), speed_m_x, &speed_x);
	*slip_x = (R_e*omega - V_x) / speed_x;

	speed_m_y = _Ca * 9.81*_t_s / (2 * Fz + _eps);
	fn_max(fabs(V_x), speed_m_y, &speed_y);
	*slip_y = V_sy / speed_y;


	fn_max(*slip_x, -1.0, &temp2);
	fn_min(temp2, 1.0, slip_x);        // combined slip�� �ǹ̸� ����ؼ� - 1, +1�� ����

	fn_max(*slip_y, -1.0, &temp1);
	fn_min(temp1, 1.0, slip_y);
}

void Obj_Tire::fn_Fiala_tire(double F[3], double M[3], double Fz, double slip, double alpha, double omega)
{
	//  _mus % mu max
	//  _mud % mu min
	//  _w   % 0.235;
	//  _rr  % m : rolling resistance
	//  _Ca  % N/rad : cornering stiffness
	//  _CSLIP % N/m : longitudinal stiffness
	double  Fx, Fy, Mzo, My, alpha_cr, sign1, sign2, sign3;
	double sl_a, mu, Scr, Fx1, Fx2, H;

	// ���ǿ� ���� Longitudinal force ��� %%%%
	if (Fz > 0) {
		sl_a = sqrt(pow(slip, 2) + pow(tan(alpha), 2));
		mu = _mus - (_mus - _mud) * sl_a;

		Scr = fabs(mu*Fz / (2 * _CSLIP));

		if (fabs(slip) < Scr) {// '='������ Fz >0 �̶�� ������ �ֱ⶧���� �ʿ����
			Fx = _CSLIP * slip; // C_Fk(k = 0�� �� Fx - k curve�� ����) * longitudinal slip ratio(k)
		}
		else {
			Fx1 = mu * Fz;
			Fx2 = fabs((mu*Fz) * (mu*Fz) / (4 * fabs(slip)*_CSLIP));
			if (slip > 0) {
				Fx = (1)*(Fx1 - Fx2);
			}
			else if (slip < 0) {
				Fx = (-1)*(Fx1 - Fx2);
			}
			else {
				Fx = 0;
			}
		}
		// ���ǿ� ���� Lateral force & Aligning Moment ��� %%%%
		alpha_cr = atan2(3 * mu*fabs(Fz), _Ca);
		if (fabs(alpha) <= alpha_cr) {
			H = 1 - (_Ca*fabs(tan(alpha)) / (3 * mu*fabs(Fz)));
			fn_sign(alpha, &sign1);
			Fy = -mu * fabs(Fz)*(1 - pow(H, 3))*sign1;

			fn_sign(alpha, &sign2);
			Mzo = mu * fabs(Fz)*_w*(1 - H)*pow(H, 3) * sign2;
		}
		else {

			fn_sign(alpha, &sign3);
			Fy = -mu * fabs(Fz)*sign3;
			Mzo = 0;
		}

		// ���ǿ� ���� Rolling Resistance Moment ��� %%%%
		// t_r = _rr*_R_u;
		// fn_step_s(omega, _slope_rr, &step);
		// My = -Fz*t_r*step; // ����:My = -_rr * Fz, �̷��� �Ǹ� ������ �־, �ڷΰ��� ����ũ�� �ɸ�
		My = 0.0;
	}
	else {        // -- - Vertical force == 0,
		Fx = 0; Fy = 0; Fz = 0;
		My = 0; Mzo = 0;
	}

	// output
	F[0] = Fx;
	F[1] = Fy;
	F[2] = Fz;
	M[0] = 0;
	M[1] = My;
	M[2] = Mzo;
}

void Obj_Tire::fn_step_s(double x, double slope, double *output) {
	// ���Ⱑ ������ step, -1, 0, 1
	// slope�� ��ȣ�ǹ� ����;
	double xc, y;

	slope = fabs(slope);
	xc = _pi / slope * 0.5;
	y = (1 - cos(slope*(x + xc))) - 1;

	if (x < -xc) { y = -1; }
	else if (x > xc) { y = 1; }
	else {}

	*output = y;
}

void Obj_Tire::fn_cross(double c[3], double a[3], double b[3])
{
	c[0] = a[1] * b[2] - a[2] * b[1];
	c[1] = a[2] * b[0] - a[0] * b[2];
	c[2] = a[0] * b[1] - a[1] * b[0];
}

void Obj_Tire::fn_mat3331(double c[3], double a[3][3], double b[3])
{
	//c(3x1)=A(3x3)*B(3x1)
	c[0] = a[0][0] * b[0] + a[0][1] * b[1] + a[0][2] * b[2];
	c[1] = a[1][0] * b[0] + a[1][1] * b[1] + a[1][2] * b[2];
	c[2] = a[2][0] * b[0] + a[2][1] * b[1] + a[2][2] * b[2];
}

void Obj_Tire::fn_mat33T31(double c[3], double a[3][3], double b[3])
{
	//c(3x1)=A(3x3)'*B(3x1)
	c[0] = a[0][0] * b[0] + a[1][0] * b[1] + a[2][0] * b[2];
	c[1] = a[0][1] * b[0] + a[1][1] * b[1] + a[2][1] * b[2];
	c[2] = a[0][2] * b[0] + a[1][2] * b[1] + a[2][2] * b[2];
}

void Obj_Tire::fn_sign(double a, double *output)
{
	double b;
	if (a > 0) { b = 1; }
	else if (a < 0) { b = -1; }
	else { b = 0; }

	*output = b;
}

void Obj_Tire::fn_max(double a, double b, double *output)
{
	double c;
	if (a >= b) { c = a; }
	else { c = b; }

	*output = c;
}

void Obj_Tire::fn_min(double a, double b, double *output)
{
	double c;
	if (a <= b) { c = a; }
	else { c = b; }

	*output = c;
}

void Obj_Tire::fn_normalize(double a[], uint num)
{
	double tmp = 0;
	uint i;
	for (i = 0; i < num; ++i)
	{
		tmp = tmp + a[i] * a[i];
	}

	for (i = 0; i < num; ++i)
	{
		a[i] = a[i] / sqrt(tmp);
	}
}

