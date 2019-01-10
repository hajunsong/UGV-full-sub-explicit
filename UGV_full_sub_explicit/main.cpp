#include "UnmannedGroundVehicle.h"

int main() {

	UnmannedGroundVehicle *ugv = new UnmannedGroundVehicle;

	ugv->init();

	ugv->equilibrium();

	ugv->run();

	delete ugv;

	return 0;
}