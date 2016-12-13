#ifndef UTIL_H
#define UTIL_H

void generate_sub_carrier(double a,double b,int n,double *e1b,double *e1c);
void generate_baseband(double *e1b,double *e1c,double base_b[][12],double base_c[][12]);

#endif // UTIL_H
