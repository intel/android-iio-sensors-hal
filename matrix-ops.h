#ifndef __MATRIX_OPS__
#define __MATRIX_OPS__

void transpose (int rows, int cols, double m[rows][cols], double m_trans[cols][rows]);
void multiply (int m, int n, int p, double m1[m][n], double m2[n][p], double result[m][p]);
void invert (int s, double m[s][s],  double m_inv[s][s]);
void multiply_scalar_inplace(int rows, int cols, double m[rows][cols], double scalar);
void assign (int rows, int cols, double m[rows][cols], double m1[rows][cols]);
void substract (int rows, int cols, double m1[rows][cols], double m2[rows][cols], double res[rows][cols]);

#endif
