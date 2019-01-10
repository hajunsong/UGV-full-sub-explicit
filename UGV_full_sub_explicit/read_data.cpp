#include "UnmannedGroundVehicle.h"

void UnmannedGroundVehicle::read_data() {
	read_system();
	read_chassis();
	read_control();
}