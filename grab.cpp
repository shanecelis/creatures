#include "animat.h"

#define POPSIZE 100
#define WINSIZE 10
#define NBTOUR 4
#define TIMEEVAL 9000


double score[2][POPSIZE];
Animat pop[2][POPSIZE];
int history[2][POPSIZE][WINSIZE];
int nbwins[2][POPSIZE];
int nbdefs[2][POPSIZE];
long int numevals;
int best[2];
int old1[2];
double oldscore;
int MAXDEF = 2;

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
		    elim->mutate();
		    break;
	    case 4: myprintf("Mutate B - \n");
		    elim->copyFrom(B);
		    elim->mutate();
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

int select(int numpop)
{
    int cur, b, n, j, winsCUR, winsB;
    cur = random() % POPSIZE;
    winsCUR = 0;
    for (j=0; j < WINSIZE; j++)
	if (history[numpop][cur][j] == VICTORY) winsCUR++;

    for (n=0; n < NBTOUR; n++)
    {
	do {b = random() % POPSIZE;} while (cur==b);
	winsB=0;
	for (j=0; j < WINSIZE; j++)
	    if (history[numpop][b][j] == VICTORY) winsB++;
	if (winsB > winsCUR) cur = b;
    }
    return cur;
}


double eval (Animat *A, Animat *B)
{
    int nbsteps, i;
    double *Apos, *Bpos, *ballpos, dist1, dist2, bestdist1, bestdist2, result;
    numevals++;
    ball.generate();
    myprintf("In Evaluation\n");
    myprintf("Animat A:\n");
    A->generate(-20*INITIALDIST,0,0);
    A->displayRepres();
    myprintf("Animat B:\n");
    B->generate(20*INITIALDIST,0,M_PI);
    B->displayRepres();
    for (nbsteps = 0; nbsteps < TIMEEVAL; nbsteps++)
    {
	doWorld(0, STEP, true);
	if (nbsteps == TIMEEVAL / 50)
	{
	    A->pushBehindX(-INITIALDIST);
	    B->pushBeforeX(INITIALDIST);
	}

    }
    ballpos = (double *)dBodyGetPosition(ball.body);
    bestdist1 = 99999; bestdist2 = 99999;
    for (i=0; i < A->nblimbs(); i++)
    {
	Apos = (double *)dBodyGetPosition(A->limbs[i].id);
	dist1 = (Apos[0] - ballpos[0])*(Apos[0] - ballpos[0])
	    + (Apos[1] - ballpos[1])*(Apos[1] - ballpos[1])
	    + (Apos[2] - ballpos[2])*(Apos[2] - ballpos[2]);
	if (dist1 < bestdist1)
	    bestdist1 = dist1;
    }
    for (i=0; i < B->nblimbs(); i++)
    {
	Bpos = (double *)dBodyGetPosition(B->limbs[i].id);
	dist2 = (Bpos[0] - ballpos[0])*(Bpos[0] - ballpos[0])
	    + (Bpos[1] - ballpos[1])*(Bpos[1] - ballpos[1])
	    + (Bpos[2] - ballpos[2])*(Bpos[2] - ballpos[2]);
	if (dist2 < bestdist2)
	    bestdist2 = dist2;
    }
    result = bestdist1 - bestdist2;
    myprintf("Result: dist1=%f, dist2=%f, score=%f\n",
	    bestdist1, bestdist2, result);
    ball.remove();
    A->remove();
    B->remove();
    return result;
}

void cycle()  
{
    static int numcycle = 0;
    int numpop;
    int i, j;
    FILE *myf;
    int new1, newbest;
    int hdefsnew, hdefsold; // defeats in recorded history
    Animat tmpA;
    char s[80];
    double score;
    int replaced, replacer;
    i=0;
    numcycle ++;

    for (numpop = 0; numpop < 2; numpop++)
    {
	do { new1 = random() % POPSIZE; } while (new1 == old1[numpop]);
	
	if (numpop == 0)
	    score = eval(&pop[0][new1], &pop[1][old1[1]]);
	else
	    score = eval(&pop[0][old1[0]], &pop[1][new1]);

	for (j=WINSIZE-1; j > 0; j--)
	{
	    history[numpop][new1][j] = history[numpop][new1][j-1];
	    history[numpop][old1[numpop]][j] = 
			history[numpop][old1[numpop]][j-1];
	}

	if (((numpop == 0) && (score < oldscore)) 
	   || ((numpop == 1) && (score > oldscore))) 
	{
	    // new1 is better
	    history[numpop][new1][0] = VICTORY;
	    history[numpop][old1[numpop]][0] = DEFEAT;
	    nbwins[numpop][new1]++;
	    nbdefs[numpop][old1[numpop]]++;
	}
	else
	{
	    history[numpop][new1][0] = DEFEAT;
	    history[numpop][old1[numpop]][0] = VICTORY;
	    nbwins[numpop][old1[numpop]]++;
	    nbdefs[numpop][new1]++;
	}

	hdefsnew = 0;
	hdefsold = 0;
	for (j=0; j < WINSIZE; j++)
	{
	    if (history[numpop][new1][j] == DEFEAT) hdefsnew++;
	    if (history[numpop][old1[numpop]][j] == DEFEAT) hdefsold++;
	}

	replaced = -1; replacer = -1;
	if ((hdefsnew >= MAXDEF) || (hdefsold >= MAXDEF))
	{
	    int A, B;
	    if (hdefsold >= MAXDEF) 
	    {
		replaced = old1[numpop];
		//replacer = new1;
	    }
	    else
		if (hdefsnew >= MAXDEF) 
		{
		    replaced = new1;
		 //   replacer = old1[numpop];
		}
		else
		    mydie("Damn ! Who should I replace ?\n"); 
	    do { A = select(numpop); } while (A == replaced);
	    do { B = select(numpop); } while ((B == replaced) || (B == A));
	    replace(&pop[numpop][replaced], &pop[numpop][A],
			&pop[numpop][B]);
	    
	    myf = fopen("creations.txt", "a");
	    fprintf(myf, "c%ldp%do%dn%d", numevals, numpop,
		    replaced, replacer);
	    for (j=0; j < WINSIZE; j++)
		if (history[numpop][replaced][j] == VICTORY) 
		    fprintf(myf, "X");
		else if  (history[numpop][replaced][j] == DEFEAT)
		    fprintf(myf, "O");
		else if  (history[numpop][replaced][j] == NONE)
		    fprintf(myf, ".");
		else
		{
		    fclose(myf);
		    mydie("Damn, neither victory nor defeat\n");
		}
	    fprintf(myf, "\n");
	    fclose(myf);
	    
	    nbdefs[numpop][replaced]=0;
	    nbwins[numpop][replaced]=0;
	    for (j=0; j < WINSIZE; j++)
		history[numpop][replaced][j] = NONE;

	}

	if (replaced != new1)
	{
	    old1[numpop] = new1;
	    oldscore = score;
	}


	newbest=0;
	for (i=0; i < POPSIZE; i++)
	    if (nbwins[numpop][i] > nbwins[numpop][newbest])
		newbest = i;
	if (newbest != best[numpop]) 
	{
	    best[numpop] = newbest;
	    myprintf("New best for pop %d !\n", numpop);
	    sprintf(s, "newbest%d.pop%d.num%d.wins%d.def%d.evals%ld.dat", 
		    numcycle,  numpop,
		    best[numpop], nbwins[numpop][best[numpop]],
		    nbdefs[numpop][best[numpop]],
		    numevals
		   );
	    pop[numpop][best[numpop]].save(s); 
	    sprintf(s, "best%d.dat", numpop);
	    pop[numpop][best[numpop]].save(s); 
	}

	
    }
}

int main (int argc, char **argv)
{
    int i, j, n;

    WORLDTYPE = FLATWORLD;
    //OUTPUTREDIRECT = TONULL;
    OUTPUTREDIRECT = TOSTDOUT;
    initWorld();
    numevals = 0;
    myprintf("Starting///\n");
    
    for (n=0; n < 2; n++)
	for (i=0; i < POPSIZE; i++)
	{
	    do{
		pop[n][i].randGenome(); 
		myprintf("Randomising %d-%d\n", n, i); 
		pop[n][i].checkGenome();
	    } while (isColliding(&pop[n][i]));
	}
    
    myprintf("Randomisation over !\n");
    
    for (n=0; n < 2; n++)
	for (i=0; i < POPSIZE; i++)
	    for (j=0; j < WINSIZE; j++)
		history[n][i][j] = NONE;
    best[0] = 0; best[1] = 0;
    old1[0] = 0; old1[1] = 0;

    oldscore=eval(&pop[0][0], &pop[1][0]);

    for (i=0; i < NBCYCLES; i++)
    {
	myprintf("Cycle %d\n", i);
	cycle();  // contrat cycle = "avant et apres, les scores sont OK" 
	/*	if (i % 50 == 0)
		{
		fclose(OUTPUTFILE);
		OUTPUTFILE = fopen ("./output", "w");
		}*/
    }
    fclose(OUTPUTFILE);
    return 0;
}
