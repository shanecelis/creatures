#ifndef _ANIMAT_EVAL_H_
#define _ANIMAT_EVAL_H_

#include "animat.h"

#define WAITTIME   1000 // Time to leave the body in simulation before, recording anything.
//#define TIMEEVAL   8000 // you can get good results with much less (eg 8-9000)
#define TIMEEVAL   40000 // you can get good results with much less (eg 8-9000)
#define PRELIMTIME  250


extern dReal epsilon;

// a <- b
void vset(dReal *a, dReal *b);
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

class Process {
public:
    virtual int timestep(int timestep, int lastTimestep, Animat *A) = 0;
};

class DistanceProcess : public Process {
public:
    dReal oldPos[3];
    dReal newPos[3];
    int timestep(int timestep, int lastTimestep, Animat *A)
    {
        dReal *pos;
        if (timestep == 0) {
            pos = A->getAvgPos();
            oldPos[0] = pos[0];
            oldPos[1] = pos[1];
            oldPos[2] = pos[2];
        } else if (timestep == lastTimestep) {
            pos = A->getAvgPos();
            newPos[0] = pos[0];
            newPos[1] = pos[1];
            newPos[2] = pos[2];

            myprintf("distanceProcess -> %f\n", distance());
        }
    }

    float distance() {
        dReal r[3];
        sub(newPos, oldPos, r);
        return norm2(r);
    }
};

int eval_callback(Animat *A, Process *f);

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

int readAnimat(char *s, Animat *a);

int endsWith(char *str, char *substring);


#endif /* _ANIMAT_EVAL_H_ */
