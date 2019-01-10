#include "UnmannedGroundVehicle.h"

void UnmannedGroundVehicle::read_system() {
	start_time = 0;
	end_time = 10;

	h = 0.01;

	g = -9.80665;

	alpha = -1 / 3.0;

	beta = ((1 - alpha)*(1 - alpha)) / 4.0;

	gamma = (1 - 2 * alpha) / 2.0;
}