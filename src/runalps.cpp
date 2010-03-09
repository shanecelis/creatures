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
#include "animat_eval.h"

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

   10    5
   20    10
   40    20
   80    40
   160   80

   5 layers, and a layersize of 60 would work for a Sims-ish run.
*/


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

int fitness_type = 0;
#define RUN 0
#define GOSTOP 1
#define UPDOWN 2
#define FOURWAY 3

void replace(Animat *elim, Animat *A, Animat *B);
double eval (Animat *A);

double randf(double start, double end)
{
    return ((rand() % 10000) * (end - start))/10000 + start;
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

double eval(Animat *A) 
{
    double a, b;
    
    switch(fitness_type) {
    case RUN:
        return prim_eval(A);
    case GOSTOP:
        goStop = 1.0;
        a = prim_eval(A);
        goStop = 0.0;
        b = prim_eval(A);
        return a/((b + 1.0)*(b + 1.0));
    case UPDOWN:
        double eval_updown(Animat *A);
        return eval_updown(A);
    case FOURWAY:
        double eval_fourway(Animat *A);
        return eval_fourway(A);
    default:
        myprintf("no fitness_type %d\n", fitness_type);
        abort();
    }
}

void setup_pop_gen(Individual* individ_config, AlpsGen* pop)
{
  pop->set_save_best(true);
  AlpsLayer layer_def;
  int age_gap = 5;
  int age_scheme = ALPS_AGING_FIBONACCI1;
  int Number_Layers = -1;
  int type = 1; // Set the population type here.
  if (type == 1) {
    // Configuration for a regular EA/GA:
    Number_Layers = 1;
    layer_def.set_select_type(ALPS_SELECT_TOURN);
    layer_def.set_size(20); //100
    layer_def.set_elitism(1); //10
    layer_def.set_tourn_size(3);
    pop->set_recomb_prob(0.5);
    pop->set_rec_rand2_prob(1.0); // 1.0
    pop->set_print_results_rate(1);//400); // 400
    pop->set_max_gen(20);
  } else if (type == 2) {
    Number_Layers = 5;
    age_gap = 5; //4
    age_scheme = ALPS_AGING_EXP;
    layer_def.set_select_type(ALPS_SELECT_TOURN);
    //    layer_def.set_select_type(ALPS_SELECT_DC);
    layer_def.set_size(40);
    layer_def.set_elitism(1);
    layer_def.set_tourn_size(9);
    layer_def.set_prob_select_prev(0.2);
    pop->set_recomb_prob(0.5);
    pop->set_rec_rand2_prob(1.0);
    pop->set_print_results_rate(1);
    pop->set_max_gen(100);
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
    //pop->write("pop.data");
  }
  // Let's save the best one.
  //Individ_Animat* ind = (Individ_Animat*) pop->get_individual(0);
  //ind->animat->save("best.json");
  printf("EA engine ended.\n");

  return 0;
}

void usage() {
    fprintf(stderr, "usage: runalps [-h] [-f type]\n");
    fprintf(stderr, "      -f fitness type \n");
    fprintf(stderr, "         0. run/go\n");
    fprintf(stderr, "         1. go stop\n");
    fprintf(stderr, "         2. up down\n");
    fprintf(stderr, "         3. four way\n");
}

void testStuff() {
    dReal r[3] = {0.0,12.0,0.0}, n[3] = {1.0,0.0,0.0};
    double f;
    f = fitness_part(r,n);
    myprintf("fitness part = %f\n", f);
    exit(1);
}

int main(int argc, char **argv) {
    //testStuff();
    int c;
    
    opterr = 0;
    
    while ((c = getopt (argc, argv, "hf:")) != -1)
        switch (c)
            {
            case 'h':
                usage();
                return 2;
            case 'f':
                fitness_type = atoi(optarg);
                break;
                
            case '?':
                if (optopt == 'f')
                    fprintf (stderr, "Option -%c requires an argument.\n", optopt);
                else if (isprint (optopt))
                    fprintf (stderr, "Unknown option `-%c'.\n", optopt);
                else
                    fprintf (stderr,
                             "Unknown option character `\\x%x'.\n",
                             optopt);
                return 1;
            default:
                abort();
            }
    
    printf("fitness_type = %d\n", fitness_type);
    WORLDTYPE = FLATWORLD;  // let's go all presocratic here...
    //OUTPUTREDIRECT = TONULL;
    initWorld();
    starttime = (long int) time(NULL);
    
    ea_engine(0);
    
    return 0;
}


