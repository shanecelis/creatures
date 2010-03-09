#ifndef _ANIMAT_EVAL_H_
#define _ANIMAT_EVAL_H_

#include "animat.h"

extern dReal epsilon;

// a `dot` b = c
dReal dot(dReal *a, dReal *b);
// a (b) = c
void scalarMult(dReal a, dReal *b, dReal *c);
// a + b = c
void add(dReal *a, dReal *b, dReal *c);
// a - b = c
void sub(dReal *a, dReal *b, dReal *c);

dReal norm2sq(dReal *a);

dReal norm2(dReal *a);

// normalize(a, a_hat) 
void normalize(dReal *a, dReal *a_hat);

int prim_eval_v(Animat *A, dReal* result);

double prim_eval(Animat *A);

double eval(Animat *A);

// u = R(a) v 
// Rotate v by the axis (0,0,1) by the angle a to produce vector u.
void rotate(dReal *u, double a, dReal *v);

double eval_updown(Animat *A);

// mean(v) = r
void mean(int n, dReal **v, dReal *r);

void stddev(int n, dReal **v, dReal *mean, dReal *std);

double eval_fourway(Animat *A);

double fitness_part(dReal *r, dReal *n);

int isColliding(Animat *elim);



#endif /* _ANIMAT_EVAL_H_ */
