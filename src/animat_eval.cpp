#include "animat_eval.h"


dReal epsilon = 0.0001;

void vset(dReal *a, dReal *b) {
    a[0] = b[0];
    a[1] = b[1];
    a[2] = b[2];
}

// a `dot` b = c
dReal dot(dReal *a, dReal *b) {
    dReal c = 0.0;
    c += a[0] * b[0];
    c += a[1] * b[1];
    c += a[2] * b[2];
    return c;
}

// a (b) = c
void scalarMult(dReal a, dReal *b, dReal *c) {
    c[0] = a * b[0];
    c[1] = a * b[1];
    c[2] = a * b[2];
}

// a + b = c
void add(dReal *a, dReal *b, dReal *c) {
    c[0] = a[0] + b[0];
    c[1] = a[1] + b[1];
    c[2] = a[2] + b[2];
}

// a - b = c
void sub(dReal *a, dReal *b, dReal *c) {
    dReal d[3];
    scalarMult(-1.0, b, d);
    add(a,d,c);
}


dReal norm2sq(dReal *a)
{
    return a[0] * a[0] + a[1] * a[1] + a[2] * a[2];
}

dReal norm2(dReal *a)
{
    return sqrt(norm2sq(a));
}

// normalize(a, a_hat) 
void normalize(dReal *a, dReal *a_hat)
{
    double mag = norm2(a);
    a_hat[0] = a[0]/mag;
    a_hat[1] = a[1]/mag;
    a_hat[2] = a[2]/mag;
}


int prim_eval_v(Animat *A, dReal* result)
{
    //cerr << "begin eval" << endl;
    int nbsteps;
    resetScene();

    //myprintf("Animat A:\n");
    A->generate(0,0,0);
    //A->setImmunityTimer(PRELIMTIME);
    A->pushBehindXVert(0);

    for (nbsteps = 0; nbsteps < WAITTIME; nbsteps++) {
        try {
            doWorld(0, STEP, true, false);
            const dReal *vel = dBodyGetLinearVel(A->limbs[0].id);
            const dReal *angvel = dBodyGetAngularVel(A->limbs[0].id);
            dReal velsq = norm2sq((dReal*) vel);
            dReal angvelsq = norm2sq((dReal*) angvel);
            //myprintf("\nvelsq: %f angvelsq: %f", velsq, angvelsq);
            if (velsq < epsilon && angvelsq < epsilon) {
                break;
            }
        } catch (...) {
            A->remove();
            result[0] = 0.0;
            result[1] = 0.0;
            result[2] = 0.0;
            return 1;
        }
    }

    //const dReal *pos = dBodyGetPosition(A->limbs[0].id);
    const dReal *pos = A->getAvgPos();
    dReal oldPos[3];
    oldPos[0] = pos[0];
    oldPos[1] = pos[1];
    oldPos[2] = pos[2];

    for (nbsteps = 0; nbsteps < TIMEEVAL; nbsteps++) {
        try {
            doWorld(0, STEP, true, true);
        } catch (...) {
            A->remove();
            result[0] = 0.0;
            result[1] = 0.0;
            result[2] = 0.0;

            return 2;
        }
    }
    
    //pos = dBodyGetPosition(A->limbs[0].id);
    pos = A->getAvgPos();
    dReal newPos[3];
    newPos[0] = pos[0];
    newPos[1] = pos[1];
    newPos[2] = pos[2];

    A->remove();

    result[0] = newPos[0] - oldPos[0];
    result[1] = newPos[1] - oldPos[1];
    result[2] = newPos[2] - oldPos[2];
    myprintf("Eval: {%f, %f, %f}\n", result[0], result[1], result[2]);
    
    return 0;
}

// typedef int (*callback)(int timestep, Animat *A, void *userdata);

// int vector_record(int timestep, Animat *A, void *userdata)
// {
//     if (timestep == 0) {
        
//     }
        
// }



int eval_callback(Animat *A, Process *f)
{
    int nbsteps;
    resetScene();

    A->generate(0,0,0);
    A->pushBehindXVert(0);

    for (nbsteps = 0; nbsteps < WAITTIME; nbsteps++) {
        try {
            doWorld(0, STEP, true, false);
            const dReal *vel = dBodyGetLinearVel(A->limbs[0].id);
            const dReal *angvel = dBodyGetAngularVel(A->limbs[0].id);
            dReal velsq = norm2sq((dReal*) vel);
            dReal angvelsq = norm2sq((dReal*) angvel);
            //myprintf("\nvelsq: %f angvelsq: %f", velsq, angvelsq);
            if (velsq < epsilon && angvelsq < epsilon) {
                break;
            }
        } catch (...) {
            A->remove();
            return 1;
        }
    }

    for (nbsteps = 0; nbsteps < TIMEEVAL; nbsteps++) {
        try {
            if (f)
                f->timestep(nbsteps, TIMEEVAL, A);
            doWorld(0, STEP, true, true);
        } catch (...) {
            A->remove();
            return 2;
        }
    }

    if (f)
        f->timestep(nbsteps, TIMEEVAL, A);

    
    A->remove();
    return 0;
}


double prim_eval(Animat *A)
{
    dReal r[3];
    prim_eval_v(A, r);
    double d = norm2(r);
    myprintf("Eval: %f\n", d);
    return d;
}

// u = R(a) v 
// Rotate v by the axis (0,0,1) by the angle a to produce vector u.
void rotate(dReal *u, double a, dReal *v)
{
    dReal t[3];
    t[0] = v[0];
    t[1] = v[1];
    t[2] = v[2];
    double cosa, sina;
    cosa = cos(a);
    sina = sin(a);
    u[0] = cosa * t[0] - sina * t[1];
    u[1] = sina * t[0] + cosa * t[1];
    u[2] = t[2];
}

double eval_updown(Animat *A) 
{
    dReal rup[3], rdown[3], rstop[3];
    upDown = 1.0;
    myprintf("Up ");
    prim_eval_v(A, rup);
    upDown = -1.0;              // Not sure if the neurons are [1,-1] or [1,0].
    myprintf("Down ");
    prim_eval_v(A, rdown);
    upDown = 0.0;
    myprintf("Stop ");
    prim_eval_v(A, rstop);
    
    // Let's project everything into the x-y plane at z = 0.
    rup[2] = rdown[2] = rstop[2] = 0.0;

    // Let a be the angle between rup and y positive, (0,1,0).  
    double a = acos(rup[1]/norm2(rup));
    dReal rupn[3], rdownn[3];

    rotate(rupn, a, rup);
    rotate(rdownn, a, rdown);
    

    double fitness = fabs(rupn[1])/(1.0 
                                    + fabs(rupn[0]) + fabs(rdownn[0]) 
                                    + norm2(rstop));
    if (rdownn[1] > 0.0) {
        fitness *= fabs(rdownn[1]);
    } else {
        fitness /= (1.0 + fabs(rdownn[1]));
    }
    myprintf("Eval_updown: %f\n", fitness);
    return fitness;
}

// mean(v) = r
void mean(int n, dReal **v, dReal *r) {
    r[0] = 0.0;
    r[1] = 0.0;
    r[2] = 0.0;
    for (int i = 0; i < n; i++) {
        add(v[i], r, r);
    }
    r[0] /= (double) n;
    r[1] /= (double) n;
    r[2] /= (double) n;
}

void stddev(int n, dReal **v, dReal *mean, dReal *std) {
    std[0] = 0.0;
    std[1] = 0.0;
    std[2] = 0.0;
    dReal tmp[3];
    for (int i = 0; i < n; i++) {
        sub(v[i], mean, tmp);
        tmp[0] *= tmp[0];
        tmp[1] *= tmp[1];
        tmp[2] *= tmp[2];
        add(std, tmp, std);
    }
    std[0] /= (double)(n - 1);
    std[1] /= (double)(n - 1);
    std[2] /= (double)(n - 1);
    std[0] = sqrt(std[0]);
    std[1] = sqrt(std[1]);    
    std[2] = sqrt(std[2]);
}

double eval_fourway(Animat *A) 
{
    dReal rup[3], rdown[3], rleft[3], rright[3], rstop[3];
    upDown = 1.0;
    leftRight = 0.0;
    myprintf("Up ");
    prim_eval_v(A, rup);
    upDown = -1.0;     // Not sure if the neurons are [1,-1] or [1,0].
    leftRight = 0.0;
    myprintf("Down ");
    prim_eval_v(A, rdown);
    upDown = 0.0;
    leftRight = 0.0;
    myprintf("Stop ");
    prim_eval_v(A, rstop);
    upDown = 0.0;
    leftRight = -1.0;
    myprintf("Left ");
    prim_eval_v(A, rleft);
    upDown = 0.0;
    leftRight = 1.0;
    myprintf("Right ");
    prim_eval_v(A, rright);
    
    // Let's project everything into the x-y plane at z = 0.
    rup[2] = rdown[2] = rstop[2] = rleft[2] = rright[2] = 0.0;

    // Let a be the angle between rup and y positive, (0,1,0).  
    double a = acos(rup[1]/norm2(rup));
    dReal n[3] = {0.0,0.0,0.0};
    double fitness_part(dReal *r, dReal *n);
    double fitness = fitness_part(rstop, n);
    
    normalize(rup, n);
    fitness *= fitness_part(rup, n);
    rotate(n, M_PI_2, n);
    fitness *= fitness_part(rleft, n);
    rotate(n, M_PI_2, n);
    fitness *= fitness_part(rdown, n);
    rotate(n, M_PI_2, n);
    fitness *= fitness_part(rright, n);

    dReal amean[3], std[3];
    dReal *v[5] = {rup, rdown, rstop, rleft, rright};
    mean(5, v, amean);
    stddev(5, v, amean, std);
    fitness *= (norm2(amean) + norm2(std));
    myprintf("Eval_fourway: %f\n", fitness);
    return fitness;
}


double fitness_part(dReal *r, dReal *n) {
    dReal d[3],e[3];
    double s, f;
    double p = 0.5;
    scalarMult(dot(r, n), n, d);
    sub(r, d, e);
    if (norm2(d) > 0.0) {
        s = dot(d, n)/norm2(d);
    } else {
        s = 0.0;
    }
    f = pow((double)(1.0 + norm2(d)), s) * pow((double)(1.0 + norm2(e)), -p);
    return f;
}

// tests for internal collisions
int isColliding(Animat *elim)
{
    NOACTUATE = 1;
    elim->generate(-50,0,0);
    if (elim->nblimbs() >= MAXLIMBS) 
        {
            myprintf("Too big !\n");
            elim->remove();
            return 1;
        }
    if (elim->test_for_intra_coll()) 
        {
            myprintf("Self-Collided - 1st pass!\n");
            elim->remove();
            return 1;
        }
    doWorld(0, STEP, true, true);
    if (elim->test_for_intra_coll()) 
        {
            myprintf("Self-Collided - 2nd pass!\n");
            elim->remove();
            return 1;
        }
    elim->remove();
    NOACTUATE=0;
    return 0;
}

int endsWith(char *str, char *substring)
{
    int l = strlen(str);
    int m = strlen(substring);
    //printf("%s\n", str + l - m);
    if (l > m) {
        return strcmp(str + l  - m, substring) == 0;
    } else {
        return 0;
    }
}

int readAnimat(char *s, Animat *a)
{
    if (endsWith(s, ".json")) {
        a->read(s);
    } else if (endsWith(s, ".bin")) {
        a->readOld(s);
    } else {
        printf("nope\n");
        return 1;
    }
    return 0;
}
