/* Header file "mrand.h" to be included by programs using mrand.c */

double mrand(int stream);
double normrand(void);

void mrandst(double* seed, int stream);
void mrandgt(double* seed, int stream);

