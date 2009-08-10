#include <time.h>
#include "sphere.h"


#define NBSPECS 2
#define SAVEPOSPERIOD 200
#define NBPOSSAVED 20
#define COEFFDAMAGE 6000000 
#define COEFFTIME 1 
#define SAVEBESTINTERVAL 5000
#define SAVEPOPINTERVAL 25000
#define INITENERGY 40000 
#define PRELIMTIME 1000


int timealive[POPSIZE];

double distcovered[POPSIZE]; //distance covered is saved but not used at present
double score[POPSIZE];
int nbkids[POPSIZE];
int spec[POPSIZE];
double volume[POPSIZE]; // the volume (bbox volume) of each creature
double volumespec[NBSPECS]; // the abg volume (bbox volume) of the species
int timebirth[NBSPECS];
int nbkillsspec[NBSPECS];
int nbkills[POPSIZE];
double savedpos[POPSIZE][NBPOSSAVED][3];
double *posA;
FILE *f;

// a bit of a hack, necessary only for the first generation of agents - see
// initialisation
int startage[POPSIZE]; 

double calcEnergy(int i)
{
    return INITENERGY  
	 - COEFFDAMAGE * pop[i].getTotalDamage() 
        - COEFFTIME * timealive[i];
/*    return INITENERGY + COEFFDIST * distcovered[i] + 
	COEFFDAMAGE * 
	(pop[i].getTotalDamageDone() - pop[i].getTotalDamage() )
        - COEFFTIME * timealive[i];*/
}

int nbIndivsInSpec(int zespec)
{
    int nb = 0;
    for (int l=0; l < POPSIZE; l++)
	if (spec[l] == zespec)
	    nb++;
    return nb;
}

int main (int argc, char **argv)
{
    int i, nbcycles=1;
    int nbdeaths=0, nbdeathsrand=0, 
	nbdeathsmaxtime=0,
	nbdeathskill=0;
    long int curID=1;
    tot_time = 0;
    long int starttime, timepast;
    OUTPUTREDIRECT = TOSTDOUT;
    WORLDTYPE = SPHERICWORLD;

    starttime = (long int) time(NULL);

    initWorld();
    
    for (i=0; i < NBSPECS; i++)
    {
	timebirth[i] = 0;
	nbkillsspec[i]=0;
    }
   
    for (i=0; i < POPSIZE; i++)
    {
	do{
	    pop[i].randGenome(); 
	    pop[i].checkGenome();
	} while (isColliding(&pop[i]));
	pop[i].ID = curID;
	pop[i].PID1 = -1; 
	pop[i].PID2 = -1; 
	curID ++;
	distcovered[i] = 0;
	nbkids[i] = 0;
	nbkills[i] = 0;
	spec[i] = random() % NBSPECS;
	timealive[i] = random() % (int)((double(INITENERGY) / COEFFTIME) - PRELIMTIME);
	startage[i] = timealive[i]; // this is necessary for the 1st batch of indivs
    }
    myprintf("Randomisation over !\n");
    
	//pop[i].read("best0.sphere.dat");
    
    dWorldSetGravity (world, 0,0,0);
    
    
    for (i=0; i < POPSIZE; i++)
    {
	pop[i].generate(50, 50, 0);
	pop[i].setImmunityTimer(PRELIMTIME);
	pop[i].dropRandomOnSphere();
    }
    

    
    
    printf("Starting algorithm..\n");
   
    f = fopen("geneal.txt","w"); 
    fclose(f);
    f = fopen("started","w"); 
    fclose(f);


    //int cpt = 0; 
    //while (cpt++ < 6)
    while (1)
    //for (int zo=0; zo < 5; zo++)
    {
	nbcycles ++;
	
	doWorld(0, STEP, true);
	for (i=0; i < POPSIZE; i++)
	    timealive[i] ++;
	
	timepast = (long int) time(NULL) - starttime;

	if (nbcycles % SAVEPOPINTERVAL == 0)
	{
	    // Just takes too much disk space
	    //char s[80];
	    //sprintf(s, "pop%06d", nbcycles / SAVEPOPINTERVAL);
	    //savePop(s);
	}
	
	
	if (nbcycles % SAVEBESTINTERVAL == 0)
	{
	    //Saves the "best" individuals of the entire simulation, and of
	    //each species, for various definitions of "best"
	    // Also sotres a few stats in "stats.txt"
	    
	    // first, save the current population to disk
	    savePop();
	    myprintf("Population saved.\n");
	    

	    dReal aabb[6];
	    for (i=0; i < POPSIZE; i++)
	    {
		dGeomGetAABB((dxGeom*)pop[i].space, aabb);
		// volume is square of volume
		volume[i] = (aabb[1]-aabb[0])*(aabb[1]-aabb[0])
		    + (aabb[3]-aabb[2])*(aabb[3]-aabb[2])
		    + (aabb[5]-aabb[4])*(aabb[5]-aabb[4]);
	    }
	    for (int l=0; l < NBSPECS; l++)
	    {
		volumespec[l]=0;
		for (i=0; i < POPSIZE; i++)
		    if (spec[i] == l)
			volumespec[l] += volume[i];
		volumespec[l] /= nbIndivsInSpec(l);
	    }
	    
	    for (i=0; i < POPSIZE; i++)
		score[i] = pop[i].getTotalDamageDone()-pop[i].getTotalDamage();
	    
	    int best1 = 0, best2 = 0;
	    best1=0;
	    for (i = 0; i < POPSIZE; i++)
		if (timealive[i] > timealive[best1]) best1 = i;
	    best2 = 0; if (best1 == 0) best2 = 1;
	    for (i = 0; i < POPSIZE; i++)
		if ((i != best1) && (timealive[i] > timealive[best2])) 
		    best2 = i;
	    pop[best1].save("bestlife0.gen.sphere.dat");
	    pop[best2].save("bestlife1.gen.sphere.dat");

	    best1=0;
	    for (i = 0; i < POPSIZE; i++)
		if (score[i] > score[best1]) best1 = i;
	    best2 = 0; if (best1 == 0) best2 = 1;
	    for (i = 0; i < POPSIZE; i++)
		if ((i != best1) && (score[i] > score[best2])) 
		    best2 = i;
	    pop[best1].save("bestfight0.gen.sphere.dat");
	    pop[best2].save("bestfight1.gen.sphere.dat");
	    
	    best1=0;
	    for (i = 0; i < POPSIZE; i++)
		if (nbkids[i] > nbkids[best1]) best1 = i;
	    best2 = 0; if (best1 == 0) best2 = 1;
	    for (i = 0; i < POPSIZE; i++)
		if ((i != best1) && (nbkids[i] > nbkids[best2])) 
		    best2 = i;
	    pop[best1].save("best0.sphere.dat");
	    pop[best2].save("best1.sphere.dat");
	    pop[best1].save("bestkids0.gen.sphere.dat");
	    pop[best2].save("bestkids1.gen.sphere.dat");
	    
	    
	    for (int sp=0; sp < NBSPECS; sp++)
	    {
		char str[80];
		best1=0;
		while (spec[best1] != sp) best1++;
		for (i = 0; i < POPSIZE; i++)
		    if ((spec[i] == sp) && (timealive[i] > timealive[best1])) 
			best1 = i;
		best2 = 0; 
		while ((best1 == best2) || (spec[best2] != sp)) best2++;
		for (i = 0; i < POPSIZE; i++)
		    if ((spec[i] == sp) && (i != best1) && 
			    (timealive[i] > timealive[best2])) 
			best2 = i;
		sprintf(str, "bestlife0.spec%d.sphere.dat", sp);
		pop[best1].save(str);
		sprintf(str, "bestlife1.spec%d.sphere.dat", sp);
		pop[best2].save(str);

		best1=0;
		while (spec[best1] != sp) best1++;
		for (i = 0; i < POPSIZE; i++)
		    if ((spec[i] == sp) && (score[i] > score[best1])) 
			    best1 = i;
		best2 = 0; 
		while ((best1 == best2) || (spec[best2] != sp)) best2++;
		for (i = 0; i < POPSIZE; i++)
		    if ((spec[i] == sp) && (i != best1) && 
		    (score[i] > score[best2])) 
			best2 = i;
		sprintf(str, "bestfight0.spec%d.sphere.dat", sp);
		pop[best1].save(str);
		sprintf(str, "bestfight1.spec%d.sphere.dat", sp);
		pop[best2].save(str);

		best1=0;
		while (spec[best1] != sp) best1++;
		for (i = 0; i < POPSIZE; i++)
		    if ((spec[i] == sp) && 
		     (nbkids[i] > nbkids[best1])) best1 = i;
		best2 = 0; 
		while ((best1 == best2) || (spec[best2] != sp)) best2++;
		for (i = 0; i < POPSIZE; i++)
		    if ((spec[i] == sp) && (i != best1) && 
		     (nbkids[i] > nbkids[best2])) 
			best2 = i;
		sprintf(str, "bestkids0.spec%d.sphere.dat", sp);
		pop[best1].save(str);
		sprintf(str, "bestkids1.spec%d.sphere.dat", sp);
		pop[best2].save(str);
	    }
	    
	    
	    f = fopen("stats.txt","w"); 
	    for (i = 0; i < POPSIZE; i++)
	    {
		fprintf(f, "%d t%d e%.4f dist%.4f dam%.4f damd%.4f drate%.4f volume%.3f kills%d kids%d spec%d\n", 
			i, timealive[i], calcEnergy(i), distcovered[i],
		       	pop[i].getTotalDamage(),
			pop[i].getTotalDamageDone(),
			((pop[i].getTotalDamageDone() - pop[i].getTotalDamage())
			/ (double)timealive[i]),
			volume[i],
			nbkills[i], nbkids[i],
			spec[i]
			);
		/*if (distcovered[i] > 4)
		{
		    fprintf(f, "%d: ", (timealive[i] - startage[i]));
		    for (int n=0; n < NBPOSSAVED; n++)
			fprintf(f, "(%.3f %.3f) ", savedpos[i][n][0],
				savedpos[i][n][1]);
		    fprintf(f, "\n");
		}*/
	    }
fprintf(f, "### Cycle %d, gen: %d deaths, %d drand,  %d dmaxtime, %d kills\n",
		    nbcycles,
		    nbdeaths, nbdeathsrand,  
		    (nbdeathsmaxtime - nbdeathsrand), nbdeathskill);
	    for (int l=0; l < NBSPECS; l++)
	fprintf(f, "# spec %d: nb %d, timebirth %d, volume %.3f, %d kills\n", 
		l, nbIndivsInSpec(l), timebirth[l], volumespec[l],
			nbkillsspec[l]);
	    fclose(f);
	
	    f = fopen("geneal.txt", "a");
	    fprintf(f,"#c %d: %dk %dd ", nbcycles / SAVEBESTINTERVAL,
		    nbdeathskill, nbdeaths);
	    for (int l=0; l < NBSPECS; l++)
		fprintf(f, "s%d:n%d s%.3f tb%d k%d; ", l, 
			nbIndivsInSpec(l), 
			volumespec[l], timebirth[l], nbkillsspec[l]);
	     fprintf(f, "\n");

	    nbdeaths=0; nbdeathsrand=0; nbdeathsmaxtime=0; 
	    nbdeathskill = 0;
	    for (int l=0; l < NBSPECS; l++) nbkillsspec[l]=0;
	    
	    fclose(f);

	}

	f = fopen("geneal.txt", "a");

	for (i=0; i< POPSIZE; i++)
	{

	    if (timealive[i] % SAVEPOSPERIOD == 0)
	    {
		// Stores the current position of each individual, to have an
		// estimate of distance covered by this individual.
		// Currently this is not used.
		for (int n=NBPOSSAVED - 1; n > 0; n -- )
		{
		    for (int k=0; k < 3; k++)
			savedpos[i][n][k] = savedpos[i][n-1][k];
		}
		posA = (double*) dBodyGetPosition(pop[i].limbs[0].id);
		for (int k=0; k < 3; k++)
		    savedpos[i][0][k] = posA[k];

		// if we're before prelimitime - ie if the animat has
		// not had enough time to fall down - then the current
		// position is copied in all saved positions
		// (note: during prelimtime creatures are also immune to
		// damage)
		// We must also put startage, because the 1st batch of indivs
		// have random initial timealive.
		if ((timealive[i] - startage[i]) <= PRELIMTIME) 
		    for (int n=0; n < NBPOSSAVED; n++)
			for (int k=0; k < 3; k++)
			    savedpos[i][n][k] = posA[k];
		
		distcovered[i] = 0;
		for (int k=0; k < 3; k++)
		    distcovered[i] += 
			(savedpos[i][0][k] - savedpos[i][NBPOSSAVED - 1][k])*
			(savedpos[i][0][k] - savedpos[i][NBPOSSAVED - 1][k]);
		distcovered[i] = sqrt(distcovered[i]);
		/*if (i == 0)
		{
		    FILE *f2 = fopen ("pos0.txt", "a");
		    fprintf(f2, "%d: ", (timealive[i] - startage[i]));
		    for (int n=0; n < NBPOSSAVED; n++)
			fprintf(f2, "(%.3f %.3f) ", savedpos[i][n][0],
				savedpos[i][n][1]);
		    fprintf(f2, "\n");
		    fclose (f2);
		}*/
	    }

	    if  (calcEnergy(i) < 0)
	    {
		// Oops, I'm dead
		
		nbdeaths ++;

		int A, B;

		// Must be replaced !
		// Choosing parents... Finding whether I have been killed, and
		// if so, which other individual gave me most damage
		
		int mostdamaging = -1;
		for (int n=0; n < POPSIZE; n++)
		{
		    if (n == i) continue;
		    if (pop[i].curDamageFrom(&pop[n]) > 0)
		    {
			if (mostdamaging == -1)
			    mostdamaging = n;
			else
			{
			    if (pop[i].curDamageFrom(&pop[n])
				    > pop[i].curDamageFrom(&pop[mostdamaging]))
				    mostdamaging = n;
			}
		    }
		}
		if (mostdamaging == -1)
		{
		    int specnew = random() % NBSPECS;
		    do {
			 A = random() % POPSIZE;
		    } while ((spec[A] != specnew) || (A == i));
		    do{
			 B = random() % POPSIZE;
		    } while ((spec[B] != specnew) || (B == i) || (A == B));
		    nbkids[A]++;
		    nbkids[B]++;
		    pop[i].remove();
		    replace(&pop[i], &pop[A], &pop[B]);
		    spec[i] = specnew;
		}
		else 
		{
		    nbdeathskill ++;
		    A = mostdamaging;
		    nbkids[A]++;
		    nbkills[A]++;
		    nbkillsspec[spec[A]]++;
		    pop[i].remove();
		    replaceWithClone(&pop[i], &pop[A]);
		    spec[i] = spec[A];
		}

		// check species numbers !!!
		for (int myspec=0; myspec < NBSPECS; myspec++)
		{
		    if (nbIndivsInSpec(myspec) < 3)
		    {
			int largestspec=0;
			for (int l = 1; l < NBSPECS; l++)
			    if (nbIndivsInSpec(l) > nbIndivsInSpec(largestspec))
				largestspec = l;
			int z=0;
			// half the largest spec is transferred to the
			// extinct spec, the last of which are eliminated
			for (int l = 0; l < POPSIZE; l++)
			{
			    if (spec[l] == myspec)
				timealive[l] += INITENERGY;
			    if (spec[l] == largestspec)
			    {
				z++;
				if (z % 2)
				    spec[l] = myspec;
			    }
			}
			timebirth[myspec] = nbcycles;
		    }
		}

		timealive[i] = 0;
		startage[i] = 0;
		nbkids[i] = 0;
		nbkills[i] = 0;
		curID ++;
		pop[i].ID = curID;
		pop[i].generate(50, 50, 
			(double)(random() % 20) * M_PI / 10.0 );
		pop[i].setImmunityTimer(PRELIMTIME);
		distcovered[i] = 0;
		pop[i].dropRandomOnSphere();
		if (pop[i].PID1 == -1)
		{
		    fprintf(f, "%d -1 %d -1 -1 %d %d", 
			    pop[i].ID,  pop[B].ID,
			    pop[B].PID1, pop[B].PID2);
		}
		else if (pop[i].PID2 == -1)
		{
		    fprintf(f, "%d %d -1 %d %d -1 -1", 
			    pop[i].ID, pop[A].ID, 
			    pop[A].PID1, pop[A].PID2);  
		}
		else
		    fprintf(f, "%d %d %d %d %d %d %d", 
			    pop[i].ID, pop[A].ID, pop[B].ID,
			    pop[A].PID1, pop[A].PID2,  
			    pop[B].PID1, pop[B].PID2);
		fprintf(f, "%d\n", spec[i]);

	    }
	}
	
	fclose(f);

	if (nbcycles % 5000 == 0) {
	    f = fopen("nbcycles", "w");
	    fprintf(f, "%d\n", nbcycles);
	    fclose(f);
	}
    }
 
    for (i=0; i < POPSIZE; i++) pop[i].remove();
    dWorldDestroy (world);
    dJointGroupDestroy (contactgroup);
    dSpaceDestroy (globalspace);
    return 0;
}
