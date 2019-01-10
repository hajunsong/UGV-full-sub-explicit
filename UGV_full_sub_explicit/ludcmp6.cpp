#include "UnmannedGroundVehicle.h"

void UnmannedGroundVehicle::ludcmp6(double a[6][6], int n, int indx[6], double d, double fac[6][6]) {
	int i, imax, j, k;
	double big, temp;
	double *vv = new double[n];
	for (i = 0; i < n; i++) {
		big = 0.0;
		for (j = 0; j < n; j++)
			if ((temp = fabs(a[i][j])) > big) big = temp;
		if (big == 0.0) {
			printf("Singular matrix in LUdcmp");
		}
		vv[i] = 1.0 / big;
	}
	for (k = 0; k < n; k++) {
		big = 0.0;
		for (i = k; i < n; i++) {
			temp = vv[i] * fabs(a[i][k]);
			if (temp > big) {
				big = temp;
				imax = i;
			}
		}
		if (k != imax) {
			for (j = 0; j < n; j++) {
				temp = a[imax][j];
				a[imax][j] = a[k][j];
				a[k][j] = temp;
			}
			d = -d;
			vv[imax] = vv[k];
		}
		indx[k] = imax;
		if (a[k][k] == 0.0) a[k][k] = TINY;
		for (i = k + 1; i < n; i++) {
			temp = a[i][k] /= a[k][k];
			for (j = k + 1; j < n; j++)
				a[i][j] -= temp * a[k][j];
		}
	}
	//////////////////
	for (i = 0; i < n; i++) {
		for (j = 0; j < n; j++) {
			fac[i][j] = a[i][j];
		}
	}

	delete[] vv;
}