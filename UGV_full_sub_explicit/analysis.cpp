#include "UnmannedGroundVehicle.h"

void UnmannedGroundVehicle::analysis() {
	orientation_chassis();
	position_chassis();
	velocity_state_chassis();
	cartesian_velocity_chassis();
	mass_force_state_chassis();

	for (int i = 0; i < 6; i++) {
		sus[i].id = i;
		orientation_suspension(&sus[i]);
		position_suspension(&sus[i]);
		velocity_state_suspension(&sus[i]);
		cartesian_velocity_suspension(&sus[i]);
	}
	if (simulation_flag == 1) {
		if (t_current == 0) {
			pre_road(sus);
		}
	}
	else {
		pre_road(sus);
	}
	for (int i = 0; i < 6; i++) {
		mass_force_state_suspension(&sus[i]);
		velocity_coupling(&sus[i]);
		effective_mass_force(&sus[i]);
	}

	acceleration_state_chassis();
	cartesian_acceleration_chassis();

	for (int i = 0; i < 6; i++) {
		acceleration_state_suspension(&sus[i]);
		cartesian_acceleration_suspension(&sus[i]);
	}

	mat33T31(chs->A0, chs->dr0c, chs->dr0cp);
	mat33T31(chs->A0, chs->ddr0c, chs->ddr0cp);
	mat33T31(chs->A0, chs->w0, chs->w0p);

	LP_control();
	wheel_spin_dyn();
}