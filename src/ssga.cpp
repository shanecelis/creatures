// (c) The university of Birmingham and quite possibly the University of
// Portsmouth, 2004-2007.
// Academic and educational uses only.


#include <time.h>
#include "animat.h"

// This program implements a Sims-like algorithm, with 2 populations and a
// "sliding archive" of champions over the last 15 generations. At every
// generation, each individual from each population is pitted against the
// current opposing champion, plus two individuals picked among the last 15
// opposing champions.

#define POPSIZE 200  // size of each species
#define NBSURV 40  // number of survivors from one generation to the next (20%)

#define TIMEEVAL 15000 // you can get good results with much less (eg 8-9000)
#define PRELIMTIME 1000

// size of the "sliding" archive of previous champions
#define SLIDINGSIZE 15 

#define NBBEST 4 // 1 current + 3 sliding archive

#define NEEDSEVAL 808

#define OVERALL  NBBEST

double score[2][POPSIZE][NBBEST + 1]; 
double tmpscore[POPSIZE][NBBEST + 1];
Animat pop[2][POPSIZE];
Animat tmppop[POPSIZE];
long int numevals;
gene hall[2][1000][MAXGENES];
int genfromhall[NBBEST];
int best[2];

long int timepast, starttime;

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
    doWorld(0, STEP, true);
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


// replaces elim's genome with either a recombination between A and B, or a
// mutated version of A or B.
void replace(Animat *elim, Animat *A, Animat *B)
{
    static int nb = 0;
    int wrong = 0;

    nb++;
    A->checkGenome(); 
    B->checkGenome();
    do{
	myprintf("* %d replacement\n", nb);

	switch (random() % 5)
	{
	    case 0: myprintf("Cross - \n");
		    if (random() % 2)
		    {
			elim->copyFrom(A);
			elim->crossWith(B); 
		    }
		    else
		    {
			elim->copyFrom(B);
			elim->crossWith(A); 
		    }
		    break;
	    case 1: myprintf("Graft - \n");
		    if (random() % 2)
		    {
			elim->copyFrom(A);
			elim->graftWith(B); 
		    }
		    else
		    {
			elim->copyFrom(B);
			elim->graftWith(A); 
		    }
		    break;
	    case 2: myprintf("Pick - \n");
		    if (random() % 2)
		    {
			elim->copyFrom(A);
			elim->pickFrom(B); 
		    }
		    else
		    {
			elim->copyFrom(B);
			elim->pickFrom(A); 
		    }
		    break;
	    case 3: myprintf("Mutate A - \n");
		    elim->copyFrom(A);
		    elim->mutate();
		    elim->mutate();
		    break;
	    case 4: myprintf("Mutate B - \n");
		    elim->copyFrom(B);
		    elim->mutate();
		    elim->mutate();
		    break;
	}

	//   else
	//	crossoverOld(0, select(), select());

	elim->displayGenomeFull();
	elim->mutate();
	elim->reassignBadConns();

	elim->checkGenome();
	elim->checkConnections();

	wrong = isColliding(elim);

    } while(wrong);
}


// Evaluation
double eval (Animat *A, Animat *B)
{
    int nbsteps;
    double result;
    numevals++;
    resetScene();
    myprintf("In Evaluation\n");
    myprintf("Animat A:\n");
    A->generate(0,0,0);
    A->setImmunityTimer(PRELIMTIME);
    A->displayRepres();
    myprintf("Animat B:\n");
    B->generate(0,0,M_PI);
    B->setImmunityTimer(PRELIMTIME);
    B->displayRepres();
    A->pushBehindXVert(0);
    B->pushBeforeXVert(0);

    for (nbsteps = 0; nbsteps < TIMEEVAL; nbsteps++)
        doWorld(0, STEP, true);

    double damageA = 0.01 + A->getTotalDamage();
    double damageB = 0.01 + B->getTotalDamage();
    result = (damageB - damageA) / (damageB + damageA);
    myprintf("Result: damage A = %f, damage B = %f, score = %f\n",
	    damageA, damageB, result);
    //A->getTotalDamage(), B->getTotalDamage(), result);

    A->remove();
    B->remove();
    return result; // in [-1; 1]; the higer, the better for A (worse for B)
}

void cycle()  
{
    static int numcycle = 0;
    int survives[POPSIZE];
    int numpop, oppop;
    int hascopiedbest;
    Animat opponent;
    int i, n, bestn;
    char s[200];
    double tot;
    int tmpbest;
    i=0;
    
    numcycle ++;

    for (numpop = 0; numpop < 2; numpop++)
    {
	oppop = 1 - numpop;

	// let us pick NBBEST-1 champions from the sliding archive for this
	// cycle
	for (i=0; i < NBBEST; i++)
	{
	    // if enough cycles have elapsed, we can ensure that the same
	    // opponent is not selected twice...
	    if (numcycle > NBBEST + 1)
	    {
		int already;
		do{
		    already=0;
		    do{
			genfromhall[i] = numcycle - SLIDINGSIZE 
			    + random() % SLIDINGSIZE;
		    }
		    while (genfromhall[i] < 0);
		    for (int j=0; j < i; j++)
			if (genfromhall[i] == genfromhall[j])
			    already = 1;
		} while (already);
	    }
	    else
	    {
		do{
		    genfromhall[i] = numcycle - 15 + random() % 15;
		}
		while (genfromhall[i] < 0);
	    }
	}
	
	
	for (i=0; i < POPSIZE; i++)
	{
	    if (numpop == 0)
	    {
		// we evaluate with the current elite opponent, then with
		// NBBEST - 1 members of the sliding hall of fame that we
		// selected earlier
		score[numpop][i][0] =
		    1 - eval(&pop[oppop][best[oppop]],
			    &pop[numpop][i]);
		for (int z=1; z < NBBEST; z++)
		{
		    for (int k=0; k < MAXGENES; k++)
			opponent.genome[k] = hall[oppop][genfromhall[z]][k];
		    score[numpop][i][z] = 
			    1 - eval(&opponent,
				    &pop[numpop][i]);
		}
	    }
	    else
	    {
		score[numpop][i][0] = 
		    1 + eval(&pop[numpop][i], 
			    &pop[oppop][best[oppop]]);
		for (int z=1; z < NBBEST; z++)
		{
		    for (int k=0; k < MAXGENES; k++)
			opponent.genome[k] = hall[oppop][genfromhall[z]][k];
		    score[numpop][i][z] = 
			    1 + eval(&pop[numpop][i], 
				    &opponent);
		}
	    }
	}
	for (i=0; i < POPSIZE; i++)
	{
	    double sum=0;
	    for (int z=0; z < NBBEST; z++)
		sum += score[numpop][i][z];
	    score[numpop][i][OVERALL] = sum / (double) NBBEST;
	}


	tmpbest = 0;

	tot=0;
	for (i=0; i < POPSIZE; i++)
	{
	    if (score[numpop][i][OVERALL] >score[numpop][tmpbest][OVERALL]) 
		tmpbest = i;
	}

	for (int k=0; k < MAXGENES; k++)
	    hall[numpop][numcycle][k] = pop[numpop][tmpbest].genome[k];

	if (tmpbest != best[numpop]) 
	{
	    best[numpop] = tmpbest;
	    sprintf(s, "best%d.dat", numpop);
	    pop[numpop][best[numpop]].save(s); 
	    timepast = (long int) time(NULL) - starttime;
	    myprintf("New best for pop %d !\n", numpop);
	    sprintf(s, 
		    "newbest%d.pop%d.num%d.score%.2f.avg%.2f.t%ld.evals%ld.dat", 
		    numcycle,  numpop, 
		    best[numpop], 
		    score[numpop][best[numpop]][OVERALL],
		    tot / (double)(POPSIZE),
		    timepast,
		    numevals
		   );
	    pop[numpop][best[numpop]].save(s); 
	}


	myprintf("Local best of pop %d is %d with overall score %f !\n", 
		numpop, best[numpop], 
		score[numpop][best[numpop]][OVERALL]);
	myprintf("Avg Score: %f\n", tot / (double)(POPSIZE));


	// Selection - Reproduction
	// 
	myprintf("Repro\n");
	for (i=0; i < POPSIZE; i++)
	    survives[i] = 0;
	for (n=0; n < NBSURV; n++) // find survivors
	{
	    bestn=0;
	    while (survives[bestn]) bestn++;
	    for (i=0; i < POPSIZE; i++)
	    {
		if ((score[numpop][i][OVERALL] > 
			    score[numpop][bestn][OVERALL]) // if i better
			&& !survives[i]) // and i is not already marked for survival
		    bestn = i;
	    }
	    survives[bestn] = 1;
	    tmppop[bestn].copyFrom(&pop[numpop][bestn]);
	    for (int k=0; k < NBBEST; k++)
		tmpscore[bestn][k] = score[numpop][bestn][k] ;
	    tmpscore[bestn][OVERALL] = score[numpop][bestn][OVERALL] ;
	}

	n=0;
	for (i=0; i < POPSIZE; i++)
	    if (survives[i])
		n++;
	if (n < NBSURV) mydie("Damn, nb of survivors is %d\n", n);

	// copying back survivors
	hascopiedbest = 0;
	tmpbest=0;
	for (n=0; n < NBSURV; n++) 	
	{
	    i=0;
	    while (!survives[i]) { i++; }
	    survives[i] = 0;
	    pop[numpop][n].copyFrom(&tmppop[i]);
	    score[numpop][n][OVERALL] = tmpscore[i][OVERALL] ;
	    for (int k=0; k < NBBEST; k++)
		score[numpop][n][k] = tmpscore[n][k];
	    if (i == best[numpop])
	    {
		myprintf("Copied best (%d) to %d\n", 
			best[numpop], n);
		tmpbest = n;
		hascopiedbest = 1;
	    }
	}
	if (!hascopiedbest)
	    mydie("Damn ! Best  didn't survive !");
	best[numpop] = tmpbest;

	// Filling the rest of the population with children of survivors
	//
	tot=0;
	for (i=0; i < NBSURV; i++)
	    tot += score[numpop][i][OVERALL];
	for (n=NBSURV; n < POPSIZE; n++) 
	{
	    // Roulette wheel selection
	    int A, B;
	    double point, acc;
	    point = random() % (int)(tot * 1000); 
	    point = point / 1000.0;
	    acc=0; 
	    for (i=0; i < NBSURV; i++)
	    {
		acc += score[numpop][i][OVERALL];
		if (acc > point) break;
	    }
	    A = i;
	    if (A >= NBSURV) mydie("Damn, couldn't select a parent A\n");

	    // So A can't be the same as B...
	    do
	    {
		point = random() % (int)(tot * 1000); 
		point = point / 1000.0;
		acc=0; 
		for (i=0; i < NBSURV; i++)
		{
		    acc += score[numpop][i][OVERALL];
		    if (acc > point) break;
		}
		B = i;
		if (B >= NBSURV) mydie("Damn, couldn't select a parent B\n");
	    }
	    while (A == B);

	    // replace n with recomb of A and B or mutated A or mutated B
	    replace (&pop[numpop][n], &pop[numpop][A], &pop[numpop][B]);

	}

	/*sprintf(s, "scorespop%d.txt", numpop);
	  FILE *f = fopen(s, "w");
	  fprintf(f, "cycle %d\n", numcycle);
	  for (i=0; i < POPSIZE; i++)
	  fprintf(f, "%d %f\n", i, score[numpop][i]);
	  fflush(f);
	  fclose(f);*/
	myprintf("Cycle done for pop %d, goto next...\n",numpop);
    }

}

int main (int argc, char **argv)
{
    int i, n;

    OUTPUTREDIRECT = TOSTDOUT;  // no output
    //OUTPUTREDIRECT = TOSTDOUT; // uncomment this to print output to stdout
    WORLDTYPE = FLATWORLD;  // let's go all presocratic here...
    initWorld();
    numevals = 0;
    starttime = (long int) time(NULL);
    myprintf("Starting///\n");

    for (n=0; n < 2; n++)
    {
	best[n] = 0; 
	for (i=0; i < POPSIZE; i++)
	{
	    do{
		pop[n][i].randGenome(); 
		myprintf("Randomising %d-%d\n", n, i); 
		pop[n][i].checkGenome();
	    } while (isColliding(&pop[n][i]));
	}
	myprintf("Randomisation over !\n");
    }
    for (n=0; n < 2; n++)
    {
	best[n]=0;
	for (i=0; i < POPSIZE; i++)
	{
	    for (int k=0; k < NBBEST; k++)
		score[n][i][k] = NEEDSEVAL;
	    score[n][i][OVERALL] = NEEDSEVAL;
	}
	for (int k=0; k < MAXGENES; k++)
	    hall[n][0][k] = pop[n][0].genome[k];
    }
    for (i=0; i < NBCYCLES; i++)
    {
	myprintf("Cycle %d\n", i);
	cycle();
	/*	if (i % 50 == 0)
		{
		fclose(OUTPUTFILE);
		OUTPUTFILE = fopen ("./output", "w");
		}*/
    }
    fclose(OUTPUTFILE);
    return 0;
}
