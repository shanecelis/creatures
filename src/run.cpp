// (c) The university of Birmingham and quite possibly the University of
// Portsmouth, 2004-2007.
// Academic and educational uses only.


#include <time.h>
#include <sys/stat.h>
#include "animat.h"
#include <math.h>

/* 
   Sims did a population size of 300 for 100 generations. It took him
   three hours on a connection machine.  What would that be for an
   ALPS approach?

   10
   20
   40
   80
   160

   5 layers, and a layersize of 60 would work for a Sims-ish run.
*/

#define TIMEEVAL   8000 // you can get good results with much less (eg 8-9000)
#define PRELIMTIME  250

#define LAYERS        5
#define LAYERSIZE    60
#define AGEGAP       10
#define TOURSIZE      9
#define ELITISM       3
#define POPSIZE       (LAYERS * LAYERSIZE)
//#define RECOMBINATION 0.5f
#define GENERATIONS 100

Animat pop[POPSIZE];
Animat tmppop[POPSIZE];
long int numevals;
long int timepast, starttime;

void replace(Animat *elim, Animat *A, Animat *B);
double eval (Animat *A);

double randf(double start, double end)
{
    return ((rand() % 10000) * (end - start))/10000 + start;
}

int layerStart(int layer)
{
    return layer * LAYERSIZE;
}

int getWorst(int layer)
{
    int idx = -1;
    for (int i = layerStart(layer); i < layerStart(layer + 1); i++) {
        if (idx == -1 || pop[i].fitness < pop[idx].fitness) 
            idx = i;
    }
    return idx;
}

int maxAge(int layer)
{
    return floor(pow(2, layer)) * AGEGAP;
}

int getBest(int layer)
{
    int idx = -1;
    for (int i = layerStart(layer); i < layerStart(layer + 1); i++) {
        if (idx == -1 || pop[i].fitness > pop[idx].fitness) 
            idx = i;
    }
    return idx;
}

int getBestOverall()
{
    int idx = -1;
    for (int i = 0; i < POPSIZE; i++) {
        if (idx == -1 || pop[i].fitness > pop[idx].fitness) 
            idx = i;
    }
    return idx;
}

int lastLayer() {
    return LAYERS - 1;
}

int tour(int layer) {
    int idx = -1;
    int lstart = layerStart(layer - 1);
    if (lstart < 0)
        lstart = 0;
    int lend = layerStart(layer + 1);
    
    for (int i = 0; i < TOURSIZE; i++) {
        int j = (rand() % (lend - lstart)) + lstart;
        if (idx == -1 || pop[idx].fitness < pop[j].fitness) 
            idx = j;
    }
    return idx;
}

void promoteAges()
{
    for (int n = 0; n < LAYERS; n++) {
        for (int i = layerStart(n); i < layerStart(n + 1); i++) {
            if (n != lastLayer() && pop[i].age >= maxAge(n)) {
                // Too old.  Check the next layer.
                int j = getWorst(n + 1);
                if (pop[j].fitness > pop[i].fitness) {
                    // The incumbent stays.
                } else {
                    // New comer takes over.
                    pop[j].copyFrom(&pop[i]);
                }
                // Oldie is tossed out.
                if (n == 0) {
                    pop[i].randGenome();
                } else {
                    replace(pop + i, pop + tour(n - 1), pop + tour(n - 1));
                }
            }
        }
    }
}

void seedPopulation()
{
    for (int i = 0; i < POPSIZE; i++) {
        pop[i].randGenome();
    }
}

void seedLayerZero() 
{
    for (int i = layerStart(0); i < layerStart(1); i++) {
        pop[i].randGenome();
    }
}

int descendFitness(const void* a, const void* b) 
{
    Animat* A = (Animat*) a;
    Animat* B = (Animat*) b;
    
    if (A->fitness > B->fitness)
        return -1;
    else if (A->fitness < B->fitness)
        return 1;
    
    return 0;
}

void rank()
{
    for (int n = 0; n < LAYERS; n++) {
        qsort(pop + layerStart(n), LAYERSIZE, sizeof(Animat), descendFitness);
    }
}

// Keep elite individual, mutate and breed the rest.
void selection(int generation)
{
    promoteAges();
    if (generation % AGEGAP == 0) {
        seedLayerZero();
    }
    rank();
    for (int n = 0; n < LAYERS; n++) {
        for (int i = layerStart(n); i < layerStart(n) + ELITISM; i++) {
            tmppop[i].copyFrom(&pop[i]);
            tmppop[i].age++;
        }
        for (int i = layerStart(n) + ELITISM; i < layerStart(n + 1); i++) {
            replace(tmppop + i, pop + tour(n), pop + tour(n));
        }
    }
    for (int i = 0; i < POPSIZE; i++) {
        pop[i].copyFrom(&tmppop[i]);
    }
}

void evaluate()
{
    for (int i = 0; i < POPSIZE; i++) {
        pop[i].fitness = eval(&pop[i]);
        //pop[i].fitness = pop[i].age;
        //pop[i].fitness = pop[i].genome[0].lgt;
    }
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

int max(int a, int b) {
    if (a > b)
        return a;
    else
        return b;
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

        //elim->displayGenomeFull();
        elim->mutate();
        elim->reassignBadConns();

        elim->checkGenome();
        elim->checkConnections();

        wrong = isColliding(elim);

    } while(wrong);
    elim->age = max(A->age, B->age) + 1;
}

double eval (Animat *A)
{
    int nbsteps;
    double result;
    numevals++;
    resetScene();
    //myprintf("In Evaluation\n");
    //myprintf("Animat A:\n");
    A->generate(0,0,0);
    //A->setImmunityTimer(PRELIMTIME);
    A->pushBehindXVert(0);

    const dReal *pos = dBodyGetPosition(A->limbs[0].id);
    dReal oldPos[3];
    oldPos[0] = pos[0];
    oldPos[1] = pos[1];
    oldPos[2] = pos[2];

    for (nbsteps = 0; nbsteps < TIMEEVAL; nbsteps++)
        doWorld(0, STEP, true, true);
    
    pos = dBodyGetPosition(A->limbs[0].id);
    dReal newPos[3];
    newPos[0] = pos[0];
    newPos[1] = pos[1];
    newPos[2] = pos[2];

    A->remove();
    result = 0.0f;
    for (int i = 0; i < 3; i++) {
        result += (oldPos[i] - newPos[i]) * (oldPos[i] - newPos[i]);
    }
    myprintf("Eval: %f\n", result);
    return result; 
    //return result; // in [-1; 1]; the higer, the better for A (worse for B)
}

int my_mkdir(char* name) {
    return mkdir(name, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
}

int main (int argc, char **argv)
{
    
    if (argc != 2) {
        fprintf(stderr, "run <result-dir>\n");
        return 2;
    }
    //OUTPUTREDIRECT = TOSTDOUT; // uncomment this to print output to stdout
    WORLDTYPE = FLATWORLD;  // let's go all presocratic here...
    int err = my_mkdir(argv[1]);
    if (err) {
        fprintf(stderr, "problem making directory '%s'\n", argv[1]);
        return 1;
    }
    char *dir = argv[1];
    char tmp[255];
    initWorld();
    numevals = 0;
    starttime = (long int) time(NULL);

    myprintf("Starting///\n");
    
    FILE *out[LAYERS];
    for (int n = 0; n < LAYERS; n++) {
        sprintf(tmp, "%s/best-fitness-l%d.data", dir, n);
        out[n] = fopen(tmp, "w");
    }
    for (int i = 0; i < POPSIZE; i++) {
        do{
            pop[i].randGenome(); 
            pop[i].checkGenome();
        } while (isColliding(&pop[i]));
    }
    myprintf("Randomisation over !\n");

    for (int i = 0; i < GENERATIONS; i++) {
        evaluate();
        for (int n = 0; n < LAYERS; n++) {
            int j = getBest(n);
            fprintf(out[n], "%f\n", pop[j].fitness);
            fflush(out[n]);
            sprintf(tmp, "%s/best-l%d-gen%.3d.dat", dir, n, i);
            pop[j].save(tmp);
        }
        selection(i);
    }
    fclose(OUTPUTFILE);
    return 0;
}
