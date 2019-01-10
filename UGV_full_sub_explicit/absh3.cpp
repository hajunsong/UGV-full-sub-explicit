#include "UnmannedGroundVehicle.h"

void UnmannedGroundVehicle::absh3(double step_size, const int n)
{
	int i;
	/*  ABSH3  : constant step Adams Bashforth 3rd order formulation.
	written by Sung-Soo Kim
	Date: Oct. 19, 1998
	copyright reserved by Sung-Soo Kim

	input variables
	t_current: current time
	Y : current state
	Yp : current derivative of state
	step_size: integration step_size

	output variables
	Y_next : state at next time step
	t_next : Next time

	STARTER:  upto 2h, i.e., derivatives are stored for the initial  time steps at 0, h, 2h, to form
	3rd order  Adams Bashforth formula */

	switch (intcount)
	{
	case 1:
		// Forward Euler method with 0.25 step_size for initial step
		// use derivative information at 0 step
		// y=y+step_size*Yp/4.0;
		for (i = 0; i < n; i++) {
			Y_next[i] = Y[i] + step_size * Yp[i] / 4.0;
		}
		// w(:,2) = Yp;
		for (i = 0; i < n; i++) {
			AW[i][1] = Yp[i];
		}
		// w1(:,2) = Yp;
		for (i = 0; i < n; i++) {
			AW1[i][1] = Yp[i];
		}
		t_next = t_current + step_size / 4.0;
		break;
	case 2:
		// Adams Bashforth 2nd order method with 0.25 step_size for 2nd step
		// use derivative inforamtion at 0, h/4
		// y = y + step_size_h * ( 3.0*Yp - w1(:,2))/8.0;
		for (i = 0; i < n; i++) {
			Y_next[i] = Y[i] + step_size * (3 * Yp[i] - AW1[i][1]) / 8.0;
		}
		// w1(:,1) = Yp;
		for (i = 0; i < n; i++) {
			AW1[i][0] = Yp[i];
		}
		t_next = t_current + step_size / 4.0;
		break;
	case 3:
		// Adams Bashforth 3rd order method with 0.25 step_size for 3rd step
		// use derivative information at 0, h/4, h/2
		// y = y + step_size * ( 23.0*Yp - 16.0*w1(:,1) + 5.0*w1(:,2))/48.0;
		for (i = 0; i < n; i++) {
			Y_next[i] = Y[i] + step_size * (23.0*Yp[i] - 16.0*AW1[i][0] + 5.0*AW1[i][1]) / 48.0;
		}
		// w1(:,2) = w1(:,1);
		for (i = 0; i < n; i++) {
			AW1[i][1] = AW1[i][0];
		}
		// w1(:,1) = Yp;
		for (i = 0; i < n; i++) {
			AW1[i][0] = Yp[i];
		}
		t_next = t_current + step_size / 4.0;
		break;
	case 4:
		// Adams Bashforth 3rd order method with 0.25 step_size for 4th step
		// use derivative information at h/4, h/2, 3h/4
		// y = y + step_size * ( 23.0*Yp - 16.0*w1(:,1) + 5.0*w1(:,2))/48.0;
		for (i = 0; i < n; i++) {
			Y_next[i] = Y[i] + step_size * (23.0*Yp[i] - 16.0*AW1[i][0] + 5.0*AW1[i][1]) / 48.0;
		}
		// w1(:,2) = w(:,2);
		for (i = 0; i < n; i++) {
			AW1[i][1] = AW[i][1];
		}
		t_next = t_current + step_size / 4.0;
		break;
	case 5:
		// Adams Bashforth 3rd order method with 0.5 step_size for 5th step
		// use derivative information at 0, h/2, h
		// y = y + step_size * ( 23.0*Yp - 16.0*w1(:,1) + 5.0*w1(:,2))/24.0;
		for (i = 0; i < n; i++) {
			Y_next[i] = Y[i] + step_size * (23.0*Yp[i] - 16.0*AW1[i][0] + 5.0*AW1[i][1]) / 24.0;
		}
		// w(:,1) = Yp;
		for (i = 0; i < n; i++) {
			AW[i][0] = Yp[i];
		}
		// w1(:,2) = w1(:,1);
		for (i = 0; i < n; i++) {
			AW1[i][1] = AW1[i][0];
		}
		// w1(:,1) = Yp;
		for (i = 0; i < n; i++) {
			AW1[i][0] = Yp[i];
		}
		t_next = t_current + step_size / 2.0;
		break;
	case 6:
		// Adams Bashforth 3rd order method with 0.5 step_size for 6th step
		// use derivative information at h/2, h,  3h/2
		// y = y + step_size * ( 23.0*Yp - 16.0*w1(:,1) + 5.0*w1(:,2))/24.0;
		for (i = 0; i < n; i++) {
			Y_next[i] = Y[i] + step_size * (23.0*Yp[i] - 16.0*AW1[i][0] + 5.0*AW1[i][1]) / 24.0;
		}
		// w1(:,2) = w1(:,1);
		for (i = 0; i < n; i++) {
			AW1[i][1] = AW1[i][0];
		}
		// w1(:,1) = Yp;
		for (i = 0; i < n; i++) {
			AW1[i][0] = Yp[i];
		}
		t_next = t_current + step_size / 2.0;
		break;
	case 7:
		// Adams Bashforth 3rd order method with step_size for 7th step
		// use derivative information at 0,  h,  2h
		// y = y + step_size * ( 23.0*Yp - 16.0*w(:,1) + 5.0*w(:,2))/12.0;
		for (i = 0; i < n; i++) {
			Y_next[i] = Y[i] + step_size * (23.0*Yp[i] - 16.0*AW[i][0] + 5.0*AW[i][1]) / 12.0;
		}
		// w(:,2) = w(:,1);
		for (i = 0; i < n; i++) {
			AW[i][1] = AW[i][0];
		}
		// w(:,1) = Yp;
		for (i = 0; i < n; i++) {
			AW[i][0] = Yp[i];
		}
		t_next = t_current + step_size;
		break;
	default:
		// Adams Bashforth 3rd order method with step_size for more than 8th step
		// use derivative information t_current-2h, t_current-h, t_current
		// y = y + step_size * ( 23.0*Yp - 16.0*w(:,1) + 5.0*w(:,2))/12.0;
		for (i = 0; i < n; i++) {
			Y_next[i] = Y[i] + step_size * (23.0*Yp[i] - 16.0*AW[i][0] + 5.0*AW[i][1]) / 12.0;
		}
		// w(:,2) = w(:,1);
		for (i = 0; i < n; i++) {
			AW[i][1] = AW[i][0];
		}
		// w(:,1) = Yp;
		for (i = 0; i < n; i++) {
			AW[i][0] = Yp[i];
		}
		t_next = t_current + step_size;
		break;
	}
	intcount++;
}