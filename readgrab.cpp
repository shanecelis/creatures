#include "animat.h"

#define TIMEEVAL 10000

//static double *trunkpos;
double XRef;
void initScene2();
Animat Ani1, Ani2;
int i;
double *Apos, *Bpos, *ballpos, dist1, dist2, bestdist1, bestdist2, result;

void simLoop2 (int pause)
{
    simLoop(pause);

    if (tot_time == TIMEEVAL / 50+1) 
    {
	Ani1.pushBehindX(-INITIALDIST);
	Ani2.pushBeforeX(INITIALDIST);
    }
    if (tot_time == TIMEEVAL)
    {
	tot_time = 0;

	ballpos = (double *)dBodyGetPosition(ball.body);
	bestdist1 = 99999; bestdist2 = 99999;
	for (i=0; i < Ani1.nblimbs(); i++)
	{
	    Apos = (double *)dBodyGetPosition(Ani1.limbs[i].id);
	    dist1 = (Apos[0] - ballpos[0])*(Apos[0] - ballpos[0])
		+ (Apos[1] - ballpos[1])*(Apos[1] - ballpos[1])
		+ (Apos[2] - ballpos[2])*(Apos[2] - ballpos[2]);
	    if (dist1 < bestdist1)
		bestdist1 = dist1;
	}
	for (i=0; i < Ani2.nblimbs(); i++)
	{
	    Bpos = (double *)dBodyGetPosition(Ani2.limbs[i].id);
	    dist2 = (Bpos[0] - ballpos[0])*(Bpos[0] - ballpos[0])
		+ (Bpos[1] - ballpos[1])*(Bpos[1] - ballpos[1])
		+ (Bpos[2] - ballpos[2])*(Bpos[2] - ballpos[2]);
	    if (dist2 < bestdist2)
		bestdist2 = dist2;
	}
	result = bestdist1 - bestdist2;

	myprintf("Result: dist1=%f, dist2=%f, score=%f\n",
		bestdist1, bestdist2, result);
	Ani1.remove();
	Ani2.remove();
	ball.remove();
	Ani1.generate(-20*INITIALDIST, 0, 0);
	Ani2.generate(20*INITIALDIST, 0, M_PI);
	ball.generate();
    }
}

void initScene2()
{
    initWorld();
    fn.step = &simLoop2;
}

int main (int argc, char **argv)
{
    tot_time = 0;
    
    //CORRIDOR = 1;
    OUTPUTREDIRECT = TOSTDOUT;
    WORLDTYPE = FLATWORLD;
    initScene2();
    Ani1.read("best0.dat");
    Ani2.read("best1.dat");
 
//    RUNREADBEST = 1;
    
    
    ball.generate();


    /*for (int i=0; i < MAXGENES; i++)
	for (int j=0; j < MAXNEUR; j++)
	    for (int k=0; k < MAXCONFROM; k++)
		Ani1.genome[i].neurons[j].confrom[k].reftype = REFBOTH;*/


    Ani1.generate(-20*INITIALDIST, 0, 0);
    Ani1.displayRepres();
    Ani2.generate(20*INITIALDIST, 0, M_PI);
    Ani2.displayRepres();
    //ball.setPos(0, INITIALDIST);
//  doWorld(0, STEP, false);
    
    printf("OK\n");
    dsSimulationLoop (argc,argv,352,288,&fn);
 
    
    dWorldDestroy (world);
    dJointGroupDestroy (contactgroup);
    dSpaceDestroy (globalspace);
    return 0;
}
