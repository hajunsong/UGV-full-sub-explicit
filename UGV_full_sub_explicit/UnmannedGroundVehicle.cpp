#include "UnmannedGroundVehicle.h"

UnmannedGroundVehicle::UnmannedGroundVehicle() {
	input = new input_data;

	chs = new chassis;
	sus = new suspension[6];
	tire = new Obj_Tire;
	ctrl = new controller;
}

UnmannedGroundVehicle::~UnmannedGroundVehicle() {
	for (int i = 0; i < input->MapInfo_Row; i++) {
		delete[] input->ElevationData[i];
	}
	delete[] input->ElevationData;

	for (int i = 0; i < input->WaypointSize; i++) {
		delete[] input->LocalPath[i];
	}
	delete[] input->LocalPath;

	delete input;
	delete chs;
	delete[] sus;
	delete tire;
	delete ctrl;
}