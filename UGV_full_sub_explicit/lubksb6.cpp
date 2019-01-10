#include "UnmannedGroundVehicle.h"

void UnmannedGroundVehicle::lubksb6(double a[6][6], int n, int indx[6], double b[6], double x[6])
{
	int i, ii = 0, ip, j;
	double sum;
	for (i = 0; i < n; i++) x[i] = b[i];
	for (i = 0; i < n; i++) {
		ip = indx[i];
		sum = x[ip];
		x[ip] = x[i];
		if (ii != 0)
			for (j = ii - 1; j < i; j++) sum -= a[i][j] * x[j];
		else if (sum != 0.0)
			ii = i + 1;
		x[i] = sum;
	}
	for (i = n - 1; i >= 0; i--) {
		sum = x[i];
		for (j = i + 1; j < n; j++) sum -= a[i][j] * x[j];
		x[i] = sum / a[i][i];
	}
}