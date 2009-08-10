#include "sphere.h"


void simLoop2(int pause)
{
    simLoop(pause);
	/*int C=0;
	double* posA; double dist;
       static double latiC, longiC;
    if (tot_time == 200)
    {
        posA = (double *) dBodyGetPosition(pop[C].limbs[0].id);
        dist = sqrt(posA[0]*posA[0] + posA[1]*posA[1] + posA[2]*posA[2]);
        latiC = acos(posA[2] / dist); // z / R
        // distance from the z axis
        dist = sqrt(posA[0]*posA[0] + posA[1]*posA[1]);
        longiC = asin(posA[0] / dist);
        if (posA[1] > 0) longiC = -longiC;
        pop[C].remove();
    }
    if (tot_time == 350)
    {
        pop[C].generate(50, 50, 0);
        pop[C].dropLatiLongi(latiC, longiC);
    }*/
}



int main (int argc, char **argv)
{
    tot_time = 0;
    int i;
    OUTPUTREDIRECT = TOSTDOUT;
    WORLDTYPE = SPHERICWORLD;


//    FREEZE = 1;
    initWorld();
    fn.step = &simLoop2;


    VISUAL = 1;
 /*  int i, j, k;
    for (i=0; i < MAXGENES; i++)
	for (j=0; j < MAXNEUR; j++)
	    for (k=0; k < MAXCONFROM; k++)
		printf("%d %d %d\n", j, k, 
			Ani.genome[i].neurons[j].confrom[k].limb);
    exit(0);*/
    //printf("OK\n");
    //Ani1.generate(-INITIALDIST, INITIALDIST, 0);
    
    
    restorePopWithPosAndVel();
    /*for (i=0; i < POPSIZE; i++)
	if (random() % 100 < 50)
	    pop[i].remove();*/
    printf("OK\n");
    dsSimulationLoop (argc,argv,352,288,&fn);
 
    for (i=0; i < POPSIZE; i++) pop[i].remove();
    dWorldDestroy (world);
    dJointGroupDestroy (contactgroup);
    dSpaceDestroy (globalspace);
    return 0;
}
