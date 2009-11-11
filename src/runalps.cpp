// (c) The university of Birmingham and quite possibly the University of
// Portsmouth, 2004-2007.
// Academic and educational uses only.


#include <time.h>
#include <sys/stat.h>
// My hand was forced on this one.  I didn't want to do it this way,
// but it would take a while to untangle the definitions from the
// implementation in animat.h, and that's obviously why it is the way
// it is because lazy wins out.

class Animat;
int isColliding(Animat *elim);
// #include "animat.h"
//#include "individ_animat.h"
#include "individ_animat.cpp"

#include "alps.h"
#include "alps_gen.h"
#include "alps_layer.h"
#include "alps_individ_real.h"
using namespace alps;

#include <cmath>
#include <cstdio>
#include <string>
using namespace std;

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

//#define TIMEEVAL   8000 // you can get good results with much less (eg 8-9000)
#define WAITTIME   1000 // Time to leave the body in simulation before, recording anything.
#define TIMEEVAL   8000 // you can get good results with much less (eg 8-9000)
#define PRELIMTIME  250

// #define LAYERS        5
// #define LAYERSIZE    60
// #define AGEGAP       10
// #define TOURSIZE      9
// #define ELITISM       3
// #define POPSIZE       (LAYERS * LAYERSIZE)
// //#define RECOMBINATION 0.5f
// #define GENERATIONS 100

long int numevals;
long int timepast, starttime;

void replace(Animat *elim, Animat *A, Animat *B);
double eval (Animat *A);

double randf(double start, double end)
{
    return ((rand() % 10000) * (end - start))/10000 + start;
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

double eval(Animat *A)
{
    //cerr << "begin eval" << endl;
    int nbsteps;
    double result;
    numevals++;
    resetScene();

    //myprintf("Animat A:\n");
    A->generate(0,0,0);
    //A->setImmunityTimer(PRELIMTIME);
    A->pushBehindXVert(0);

    for (nbsteps = 0; nbsteps < WAITTIME; nbsteps++) {
        try {
            doWorld(0, STEP, true, true);
        } catch (...) {
            A->remove();
            return 0.0f;
        }
    }

    const dReal *pos = dBodyGetPosition(A->limbs[0].id);
    dReal oldPos[3];
    oldPos[0] = pos[0];
    oldPos[1] = pos[1];
    oldPos[2] = pos[2];

    for (nbsteps = 0; nbsteps < TIMEEVAL; nbsteps++) {
        try {
            doWorld(0, STEP, true, true);
        } catch (...) {
            A->remove();
            return 0.0f;
        }
    }
    
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
    //cerr << "end eval" << endl;
    return result; 
    //return result; // in [-1; 1]; the higer, the better for A (worse for B)
}

int my_mkdir(char* name) {
    return mkdir(name, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
}

int main2 (int argc, char **argv)
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
    return 0;
}

bool evaluate_individ(vector<double>& fitness, Individual* individ)
{
  if (individ == NULL) {
    cerr << "evo_real :: evaluate_individ() - error, null individ sent.\n";
    while (1) ;
    return false;
  }

  Individ_Animat* ind = (Individ_Animat*) individ;
  fitness[0] = eval(ind->animat);
  return true;
}

void setup_pop_gen(Individual* individ_config, AlpsGen* pop)
{
  pop->set_save_best(true);
  AlpsLayer layer_def;
  int age_gap = 5;
  int age_scheme = ALPS_AGING_FIBONACCI1;
  int Number_Layers = -1;
  int type = 2; // Set the population type here.
  if (type == 1) {
    // Configuration for a regular EA/GA:
    Number_Layers = 1;
    layer_def.set_select_type(ALPS_SELECT_TOURN);
    layer_def.set_size(10); //100
    layer_def.set_elitism(1); //10
    layer_def.set_tourn_size(5);
    pop->set_recomb_prob(0.5);
    pop->set_rec_rand2_prob(1.0); // 1.0
    pop->set_print_results_rate(1);//400); // 400
    pop->set_max_gen(10);
  } else if (type == 2) {
    Number_Layers = 5;
    age_gap = 10; //4
    age_scheme = ALPS_AGING_EXP3;
    layer_def.set_select_type(ALPS_SELECT_TOURN);
    //    layer_def.set_select_type(ALPS_SELECT_DC);
    layer_def.set_size(60);
    layer_def.set_elitism(3);
    layer_def.set_tourn_size(9);
    layer_def.set_prob_select_prev(0.2);
    pop->set_recomb_prob(0.5);
    pop->set_rec_rand2_prob(1.0);
    pop->set_print_results_rate(1);
    pop->set_max_gen(20);
  } else {
    cerr << "tiny :: setup_pop_gen() - error, invalid EA type: "
	 << type << "\n";
    return;
  }

  pop->config_layers_same(age_scheme, age_gap,
			  Number_Layers, layer_def);
  pop->print_layers();
  pop->set_num_runs(1);
  pop->set_maximize();

}

void *ea_engine(void *arg1)
{
  cout << "EA engine started.\n";

  Individ_Animat *individ_config = new Individ_Animat();

  vector<double> fitness;
  fitness.resize(1);

  // Configure a generational ALPS population:
  Alps *pop = new AlpsGen("animat", individ_config);
  setup_pop_gen(individ_config, (AlpsGen*)pop);

  pop->set_print_debug(true);
  pop->write_header(cout);

  while (! pop->is_finished()) {
    int index;
    Individual* individ;
    int res = pop->get_next_individ(index, individ);
    if (res == -1) {
      continue; // Get another index / Try again.

    } else if (res == -2) {
      // Evolution is over.
      break;
    }

    vector<double> fitness;
    fitness.resize(1);
    int result = evaluate_individ(fitness, individ);
    if (result == false) {
      // Error evaluating this individual.
      pop->evaluate_error(index, individ);
    } else {
      // Evaluated successfully.
      pop->insert_evaluated(fitness, index, individ, 0);
    }
    // How do I know when a generation is over?
  }
  // Let's save the best one.
  Individ_Animat* ind = (Individ_Animat*) pop->get_individual(0);
  ind->animat->save("best.json");
  printf("EA engine ended.\n");

  return 0;
}

int main(int argc, char **argv) {

    WORLDTYPE = FLATWORLD;  // let's go all presocratic here...
    initWorld();
    starttime = (long int) time(NULL);
    
    ea_engine(0);
    
    return 0;
}


