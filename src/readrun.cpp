#define DRAWSTUFF
#include "animat.h"

#define TIMEEVAL 10000
#define PRELIMTIME 1000

//static double *trunkpos;
double XRef;
void initScene2();
Animat Ani1;
int i;
double *Apos, *Bpos, dist1, dist2, bestdist1, bestdist2, result;

void simLoop2 (int pause)
{
    simLoop(pause);

    if (tot_time == TIMEEVAL) {
            tot_time = 0;

            bestdist1 = Ani1.getTotalDamage() + 0.01;
            bestdist2 = 0.0;
            //bestdist2 = Ani2.getTotalDamage() + 0.01;
            result = (bestdist2 - bestdist1) / 
                (bestdist2 + bestdist1);

            myprintf("\nResult: damage1=%f, damage2=%f, score=%f\n",
                     bestdist1, bestdist2, result);

            Ani1.remove();
            resetScene();
            Ani1.generate(0, 0, 0);
            Ani1.setImmunityTimer(PRELIMTIME);
            Ani1.pushBehindXVert(0);
    }
}

void initScene2()
{
    initWorld();
    fn.step = &simLoop2;
}

int main (int argc, char **argv)
{

    if (argc != 2) {
        fprintf(stderr, "usage: readrun <file.dat>\n");
        return 2;
    }
    tot_time = 0;
    
    //CORRIDOR = 1;
    OUTPUTREDIRECT = TOSTDOUT;
    WORLDTYPE = FLATWORLD;
    BALL = 0;
    initScene2();
    Ani1.read(argv[1]);
 
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
