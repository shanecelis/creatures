#include "animat.h"

#define POPSIZE 100

Animat pop[POPSIZE];



void dropPopEven()
{
// drops the individuals evenly over the sphere.
// must only be called AFTER all individuals have been generated
// Rakhmanov, Saff and Zhou's spiral algorithm, as described in 
// Saff and Kuijlaars "Distributing many points on a sphere",
// http://sitemason.vanderbilt.edu/page/hmbADS
    double th, ph = 0, h;
    pop[0].dropLatiLongi(0, 0);
    for (int i=1; i < POPSIZE; i++)
    {
	h = (2.0 * i) / (POPSIZE) - 1.0;
	if (i == POPSIZE - 1)
	    ph = 0;
	else
	    ph = ph + 3.9 / (.00001 + ( sqrt(POPSIZE + 1.0) * sqrt(1 - h*h) ));
	th = acos(h);
	if (ph > 2 * M_PI) ph -= 2 * M_PI;
	//myprintf("%d: %.4f, %.4f\n", i, th, ph);
	pop[i].dropLatiLongi(th, ph);
    }
}


void savePop(const char *filename)
{
    FILE *f = fopen(filename, "w");
    for (int i=0; i < POPSIZE; i++)
    {
	fwrite(pop[i].genome, 1, sizeof(pop[i].genome), f);
	for (int j=0; j < pop[i].nblimbs(); j++)
	{
	    fwrite(dBodyGetPosition(pop[i].limbs[j].id), 3, sizeof(dReal), f);
	    fwrite(dBodyGetQuaternion(pop[i].limbs[j].id), 1, 
		    sizeof(dQuaternion), f);
	    fwrite(dBodyGetLinearVel(pop[i].limbs[j].id), 3, 
			sizeof(dReal), f);
	    fwrite(dBodyGetAngularVel(pop[i].limbs[j].id), 3, 
			sizeof(dReal), f);
	    fwrite(&pop[i].limbs[j].immunitytimer, 1, sizeof(int), f);
	    //fwrite(&pop[i].limbs[j].damagedone, 1, sizeof(double), f);
	    //fwrite(&pop[i].limbs[j].damage, 1, sizeof(double), f);
	}
    }
    fflush(f);
    fclose(f);
}


void savePop()
{
    FILE *f = fopen("pop.sphere.dat", "w");
    for (int i=0; i < POPSIZE; i++)
    {
	fwrite(pop[i].genome, 1, sizeof(pop[i].genome), f);
	for (int j=0; j < pop[i].nblimbs(); j++)
	{
	    fwrite(dBodyGetPosition(pop[i].limbs[j].id), 3, sizeof(dReal), f);
	    fwrite(dBodyGetQuaternion(pop[i].limbs[j].id), 1, 
		    sizeof(dQuaternion), f);
	    fwrite(dBodyGetLinearVel(pop[i].limbs[j].id), 3, 
			sizeof(dReal), f);
	    fwrite(dBodyGetAngularVel(pop[i].limbs[j].id), 3, 
			sizeof(dReal), f);
	    fwrite(&pop[i].limbs[j].immunitytimer, 1, sizeof(int), f);
	    //fwrite(&pop[i].limbs[j].damagedone, 1, sizeof(double), f);
	    //fwrite(&pop[i].limbs[j].damage, 1, sizeof(double), f);
	}
    }
    fflush(f);
    fclose(f);
}


void restorePopGenomeOnly()
{
    long int dummytimer;
    FILE *f = fopen("pop.sphere.dat", "r");
    dReal tmppos[3];
    dReal tmplinvel[3];
    dReal tmpangvel[3];
    dQuaternion tmpquat;
    //double tmpdamage;
    for (int i=0; i < POPSIZE; i++)
    {
	fread(pop[i].genome, 1, sizeof(pop[i].genome), f);
	// necessary to get nblimbs() to return the right val:
	pop[i].generate(0, 0, 0);
	for (int j=0; j < pop[i].nblimbs(); j++)
	{
	    fread(tmppos, 3, sizeof(dReal), f);
	    fread(tmpquat, 1,sizeof(dQuaternion), f);
	    fread(tmplinvel, 3, sizeof(dReal), f);
	    fread(tmpangvel, 3, sizeof(dReal), f);
	    fread(&dummytimer, 1, sizeof(int), f);
	    //fread(&tmpdamage, 1, sizeof(double), f);
	    //fread(&tmpdamage, 1, sizeof(double), f);
	}
	pop[i].remove();
    }
    fclose(f);
}


void restorePop()  
{
    FILE *f = fopen("pop.sphere.dat", "r");
    for (int i=0; i < POPSIZE; i++)
    {
	fread(pop[i].genome, 1, sizeof(pop[i].genome), f);
	pop[i].generate(0, 0, 0);
	for (int j=0; j < pop[i].nblimbs(); j++)
	{
	    dReal savedpos[3];
	    dReal savedlinvel[3];
	    dReal savedangvel[3];
	    dQuaternion savedquat;
	    fread(savedpos, 3, sizeof(dReal), f);
	    fread(savedquat, 1,sizeof(dQuaternion), f);
	    fread(savedlinvel, 3, sizeof(dReal), f);
	    fread(savedangvel, 3, sizeof(dReal), f);
	    fread(&pop[i].limbs[j].immunitytimer, 1, sizeof(int), f);
	    //fread(&pop[i].limbs[j].damagedone, 1, sizeof(double), f);
	    //fread(&pop[i].limbs[j].damage, 1, sizeof(double), f);
	}
    }
    fclose(f);
}


void restorePopWithPosAndVel()
{
    FILE *f = fopen("pop.sphere.dat", "r");
    for (int i=0; i < POPSIZE; i++)
    {
	fread(pop[i].genome, 1, sizeof(pop[i].genome), f);
	pop[i].generate(0, 0, 0);
	for (int j=0; j < pop[i].nblimbs(); j++)
	{
	    dReal savedpos[3];
	    dReal savedlinvel[3];
	    dReal savedangvel[3];
	    dQuaternion savedquat;
	    fread(savedpos, 3, sizeof(dReal), f);
	    fread(savedquat, 1,sizeof(dQuaternion), f);
	    fread(savedlinvel, 3, sizeof(dReal), f);
	    fread(savedangvel, 3, sizeof(dReal), f);
	    fread(&pop[i].limbs[j].immunitytimer, 1, sizeof(int), f);
	    
	    dBodySetPosition(pop[i].limbs[j].id, savedpos[0], 
		    savedpos[1], savedpos[2]);
	    dBodySetQuaternion(pop[i].limbs[j].id, savedquat);
	    dBodySetLinearVel(pop[i].limbs[j].id, savedlinvel[0], 
		    savedlinvel[1], savedlinvel[2]);
	    dBodySetAngularVel(pop[i].limbs[j].id, savedangvel[0], 
		    savedangvel[1], savedangvel[2]);
	    //fread(&pop[i].limbs[j].damagedone, 1, sizeof(double), f);
	    //fread(&pop[i].limbs[j].damage, 1, sizeof(double), f);
	}
    }
    fclose(f);
}


int isColliding(Animat *elim)
{
    NOACTUATE = 1;
    elim->generate(-50,-50,0);
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
    myprintf("Doing world...\n ");
    doWorld(0, STEP, true);
    myprintf("world done.\n");
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



void replaceWithClone(Animat *elim, Animat *A)
{
    int wrong = 0;
    myprintf("In ReplClo..\n");

    do{
	elim->copyFrom(A);
	elim->mutate();
	elim->mutate();
	elim->mutate();
	elim->reassignBadConns();

	//elim->displayGenomeFull();
	elim->checkGenome();
	elim->checkConnections();

	wrong = isColliding(elim);

    } while(wrong);
    elim->PID1 = A->ID;
    elim->PID2 = -1;
}


void replace(Animat *elim, Animat *A, Animat *B)
{
    static int nb = 0;
    int wrong = 0;

	nb++;
    //A->checkGenome(); 
    //B->checkGenome();
    do{
	myprintf("* %d replacement\n", nb);
	
	switch (random() % 6)
	{
	    case 0: myprintf("Cross - \n");
			elim->copyFrom(A);
			elim->crossWith(B); 
		    elim->PID1 = A->ID;
		    elim->PID2 = B->ID;
		    break;
	    case 1: myprintf("Cross - \n");
			elim->copyFrom(B);
			elim->crossWith(A); 
		    elim->PID1 = B->ID;
		    elim->PID2 = A->ID;
		    break;
	    case 2: myprintf("Graft - \n");
			elim->copyFrom(A);
			elim->graftWith(B); 
		    elim->PID1 = A->ID;
		    elim->PID2 = B->ID;
		    break;
	    case 3: myprintf("Graft - \n");
			elim->copyFrom(B);
			elim->graftWith(A); 
		    elim->PID1 = B->ID;
		    elim->PID2 = A->ID;
		    break;
	    case 4: myprintf("Mutate A - \n");
		    elim->copyFrom(A);
		    elim->mutate();
		    elim->mutate();
		    elim->PID1 = A->ID;
		    elim->PID2 = -1;
		    break;
	    case 5: myprintf("Mutate B - \n");
		    elim->copyFrom(B);
		    elim->mutate();
		    elim->mutate();
		    elim->PID1 = -1;
		    elim->PID2 = B->ID;
		    break;
	}

	//   else
	//	crossoverOld(0, select(), select());


	
	elim->mutate();
	elim->reassignBadConns();
	
	//elim->displayGenomeFull();
	elim->checkGenome();
	elim->checkConnections();
    
	wrong = isColliding(elim);

    } while(wrong);
}

