#include "animat.h"
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>


#define WAITTIME 1000
#define TIMEEVAL 10000
#define PRELIMTIME 1000

#define SETTLEDMAX 10
//static double *trunkpos;
double XRef;
void initScene2();
Animat Ani1;
int i;
int brains = 1;
int fitness_type = 0;
int timeout = 0;
int settledCount = 0;
double *Apos, *Bpos, dist1, dist2, bestdist1, bestdist2, result;
dReal oldPos[3];
dReal newPos[3];

typedef enum { drop, settled, go, record } SimState;
SimState state = drop;
dReal epsilon = 0.0001;

dReal norm2sq(const dReal *a)
{
    return a[0] * a[0] + a[1] * a[1] + a[2] * a[2];
}

void simLoop2 (int pause)
{

    
    if (state == drop) {
        // Run without brains.
        simLoopPlus(pause, false);
        const dReal *vel = dBodyGetLinearVel(Ani1.limbs[0].id);
        const dReal *angvel = dBodyGetAngularVel(Ani1.limbs[0].id);
        dReal velsq = norm2sq(vel);
        dReal angvelsq = norm2sq(angvel);
        //myprintf("\nvelsq: %f angvelsq: %f", velsq, angvelsq);
        if (velsq < epsilon && angvelsq < epsilon) {
            settledCount++;
        }
        if (settledCount >= SETTLEDMAX) {
            state = settled;
            state = go;
            //const dReal *pos = dBodyGetPosition(Ani1.limbs[0].id);
            const dReal *pos = Ani1.getAvgPos();
            oldPos[0] = pos[0];
            oldPos[1] = pos[1];
            oldPos[2] = pos[2];
            tot_time = 0;
            myprintf("\nsettled.\n");
        }
    } else {
        // Turn the brains on.
        simLoopPlus(pause, brains == 1);
        //const dReal *pos = dBodyGetPosition(Ani1.limbs[0].id);
        const dReal *pos = Ani1.getAvgPos();
        newPos[0] = pos[0];
        newPos[1] = pos[1];
        newPos[2] = pos[2];
        dsDrawLine(oldPos, newPos);

        if (tot_time == TIMEEVAL) {
            
            dReal result = 0.0f;
            for (int i = 0; i < 3; i++) {
                result += (oldPos[i] - newPos[i]) * (oldPos[i] - newPos[i]);
            }
            myprintf("\nResult: %f\n", result);

            if (timeout) {
                tot_time = 0;
                exit(0);
                
                Ani1.remove();
                resetScene();
                Ani1.generate(0, 0, 0);
                //Ani1.setImmunityTimer(PRELIMTIME);
                Ani1.pushBehindXVert(0);
                state = drop;
            }
        }
    }
}

void keypressed(int cmd) 
{
    switch (cmd) {
    case 'w':
        if (upDown == 1.0) {
            myprintf("\nstop\n");
            upDown == 0.0;
        } else {
            upDown = 1.0;
            myprintf("\nup\n");
        }
        break;
    case 's':
        if (upDown == -1.0) {
            myprintf("\nstop\n");
            upDown == 0.0;
        } else {
            upDown = -1.0;
            myprintf("\ndown\n");
        }
        break;
    case 'a':
        myprintf("\nleft\n");
        break;
    case 'd':
        myprintf("\nright\n");
        break;
    case 'g':
        if (goStop == 0.0) {
            goStop = 1.0;
            myprintf("\ngo\n");
        } else {
            goStop = 0.0;
            myprintf("\nstop\n");
        }
        break;
    case 'b':
        if (brains) {
            brains = 0;
            myprintf("\nbrains off\n");
        } else {
            brains = 1;
            myprintf("\nbrains on\n");
        }
    }
    
}

void initScene2()
{
    initWorld();
    fn.step = &simLoop2;
    fn.command = &keypressed;
    goStop = 1.0;
    upDown = 0.0;
}

void usage()
{
    fprintf(stderr, "usage: readrun [-ft] <file.dat>\n");
    fprintf(stderr, "       -f fitness type\n");
    fprintf(stderr, "       -t timeout\n");
}

int main (int argc, char **argv)
{
    int c;
    opterr = 0;
    
    while ((c = getopt (argc, argv, "tf:")) != -1)
        switch (c) {
        case 'f':
            fitness_type = atoi(optarg);
            break;
        case 't':
            timeout = 1;
            break;
        case '?':
            if (optopt == 'f')
                fprintf (stderr, "Option -%c requires an argument.\n", optopt);
            else if (isprint (optopt))
                fprintf (stderr, "Unknown option `-%c'.\n", optopt);
            else
                fprintf (stderr,
                         "Unknown option character `\\x%x'.\n",
                         optopt);
            return 1;
        default:
            abort();
        }
    argv += optind;
    argc -= optind;

    if (argc != 1) {
        usage();
        return 2;
    }
    tot_time = 0;
    
    //CORRIDOR = 1;
    OUTPUTREDIRECT = TOSTDOUT;
    WORLDTYPE = FLATWORLD;
    BALL = 0;
    initScene2();
    Ani1.read(argv[0]);
 
    VISUAL = 1;
    
    
    /*for (int i=0; i < MAXGENES; i++)
      for (int j=0; j < MAXNEUR; j++)
      for (int k=0; k < MAXCONFROM; k++)
      Ani1.genome[i].neurons[j].confrom[k].reftype = REFBOTH;*/

    resetScene();

    Ani1.generate(0, 0, 0);
    Ani1.setImmunityTimer(PRELIMTIME);
    Ani1.displayRepres();
	Ani1.pushBehindXVert(0);
    
    printf("OK\n");
    dsSimulationLoop (argc,argv,352,288,&fn);
    destroyWorld();
    return 0;
}
