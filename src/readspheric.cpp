#include "animat.h"

#define TIMEEVAL 10000
#define PRELIMTIME 1000

static double *trunkpos;
double XRef;
void initScene2();
Animat Ani1, Ani2;
int i;
double *Apos, *Bpos, dist1, dist2, bestdist1, bestdist2, result;

void simLoop2 (int pause)
{
    simLoop(pause);

    if (tot_time == TIMEEVAL)
    {
	tot_time = 0;

	bestdist1 = Ani1.getTotalDamage() + 0.01;
	bestdist2 = Ani2.getTotalDamage() + 0.01;
	result = (bestdist2 - bestdist1) / 
	    (bestdist2 + bestdist1);

	myprintf("\nResult: damage1=%f, damage2=%f, score=%f\n",
		bestdist1, bestdist2, result);

	Ani1.remove();
	Ani2.remove();
	resetScene();
	Ani1.generate(0, 0, 0);
	Ani1.setImmunityTimer(PRELIMTIME);
	Ani2.generate(0, 0, 0);
	Ani2.setImmunityTimer(PRELIMTIME);
	Ani1.dropLatiLongi(M_PI / 10, 0);
	Ani2.dropLatiLongi(M_PI / 10, M_PI);
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
    WORLDTYPE = SPHERICWORLD;
    initScene2();
    Ani1.read("best0.sphere.dat");
    Ani2.read("best1.sphere.dat");
    if (argc == 3)
    {
	Ani1.read(argv[1]);
	Ani2.read(argv[2]);
    }

    if (argc == 2)
    {
	if (strstr(argv[1], "0"))
	{
	    Ani1.read("best0.sphere.dat");
	    Ani2.read("best1.sphere.dat");
	}
	if (strstr(argv[1], "1"))
	{
	    Ani1.read("best1.sphere.dat");
	    Ani2.read("best2.sphere.dat");
	}
	if (strstr(argv[1], "2"))
	{
	    Ani1.read("best2.sphere.dat");
	    Ani2.read("best0.sphere.dat");
	}
    }
 
    VISUAL = 1;
    
    
    /*for (int i=0; i < MAXGENES; i++)
	for (int j=0; j < MAXNEUR; j++)
	    for (int k=0; k < MAXCONFROM; k++)
		Ani1.genome[i].neurons[j].confrom[k].reftype = REFBOTH;*/

    resetScene();

    Ani1.generate(0, 0, 0);
    Ani1.setImmunityTimer(PRELIMTIME);
    Ani1.displayRepres();
    Ani2.generate(0, 0, M_PI);
    Ani2.setImmunityTimer(PRELIMTIME);
    Ani2.displayRepres();
	Ani1.dropLatiLongi(M_PI / 20, 0);
	Ani2.dropLatiLongi(M_PI / 20, M_PI);

    
    printf("OK\n");
    dsSimulationLoop (argc,argv,352,288,&fn);
    destroyWorld();
    return 0;
}
