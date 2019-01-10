#include "UnmannedGroundVehicle.h"

void UnmannedGroundVehicle::pre_road(suspension *sus) {
	double road_h[6];

	for (int i = 0; i < 6; i++) {
		//Obj_Map::PNU_fn_map(sus[i].rw[0], sus[i].rw[1], &road_h[i]);
		road_h[i] = 0;
	}

	double P_RF[3] = { sus[RF].rw[0], sus[RF].rw[1], road_h[0] };
	double P_RR[3] = { sus[RR].rw[0], sus[RR].rw[1], road_h[2] };
	double P_LF[3] = { sus[LF].rw[0], sus[LF].rw[1], road_h[3] };
	double P_LR[3] = { sus[LR].rw[0], sus[LR].rw[1], road_h[5] };

	double T_vec1[3], T_vec2[3], T_vec1t[3][3], N_vec[3];
	for (int i = 0; i < 3; i++) {
		T_vec1[i] = P_RF[i] - P_LR[i];
		T_vec2[i] = P_LF[i] - P_RR[i];
	}

	tilde(T_vec1, T_vec1t);
	mat3331(T_vec1t, T_vec2, N_vec);

	// vector normalization
	double tmp = 0;
	for (int i = 0; i < 3; ++i) tmp = tmp + N_vec[i] * N_vec[i];
	for (int i = 0; i < 3; ++i) N_vec[i] = N_vec[i] / sqrt(tmp);

	tire->set_Road_h(road_h);
	tire->set_N_vec(N_vec);
}