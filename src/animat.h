// (c) The university of Birmingham and quite possibly the University of
// Portsmouth, 2004-2007.
// Academic and educational uses only.


//#define DRAWSTUFF 1
#include <ode/ode.h>
#ifdef DRAWSTUFF
#include <drawstuff/drawstuff.h>
#endif
#include <stdlib.h>
#include <time.h>
#include <json/json.h>

// Refuses conns from actuators !
// But allows them from self !

// select correct drawing functions
#ifdef dDOUBLE
#define dsDrawSphere dsDrawSphereD
#define dsDrawBox dsDrawBoxD
#endif


// Note: Many of the following constants are actually not used in the currents
// version of the code.

// ODE constants, obtained through painful parameter tuning
#define MYCFM 0.01
#define MYERP 0.015


// MAX NUMBER OF ANIMATS ALIVE IN THE SIMULATION AT ANY TIME
// Increasing this much beyond need may reduce performance
// Putting more animats in the simulation will break the system
#define REGISTSIZE  100


#define PROBAMUT  2//4 // probability of mutation - TRY TO INCREASE THIS !
#define WORLDRADIUS 15 // for spherical world
#define DROPHEIGHT 10 // for spherical world
#define HURTIMMUNITYTIMER 200
#define VICTORY 123
#define DEFEAT 321
#define NONE 444
#define SPEEDMULT 6.0
#define MAXTHRES 0.5
#define MAXRECUR 3
#define NBCYCLES 10000000
#define INITIALDIST 1.5
#define MYRANDSEED 0 //  if 0, then random seed is taken from /dev/urandom
#define SIDE (1.0f)	// side length of a box
#define MAXFORCE 4 // 3 ?
#define NBNEURCYCLES 2  // 2 ?
#define STEP 0.01
#define PROBAINITRECUR 25 // probability of recursion in initial random genome
#define PROBAINITREF 25 // same for reflection 
#define MINGENES 3 // minimum number of genes allowed in a genome
#define MAXGENES 8 // maximum
#define MAXLIMBS 18 // maximum number of limbs allowed after development
#define CORRSPACE 12
#define WALLSIZE 150
#define MAXCONFROM 3
#define MAXCONT 4
#define BSIDE 24
#define SQSIZE 8
#define PROBANEUREXISTS 66 // 50
#define PROBANEURSENS 50 // 50, with 2 sensor types
#define PROBACONNEXISTS 90
#define MAXNEUR 5

#define SIGMOID 10
#define INVEXP 20
#define TANH 30

#define INTER 0
#define ACTUATOR 1
#define SENSPROP 2
#define SENSBALLX 3
#define SENSBALLY 4
#define SENSCLOSESTANIMY 5
#define SENSCLOSESTANIMX 6
#define SENSTOUCH 7
#define SENSDAMAGE 8
#define SENSUPDOWN 9
#define SENSLEFTRIGHT 10

#define TOFILE 54
#define TONULL 55
#define TOSTDOUT 56

#define REFBOTH 44
#define REFORIG 45
#define REFSYMM 46

#define RECSELF 90
#define RECDAD 91
#define RECSON 92

#define SPHERICWORLD 505
#define FLATWORLD 506


// forces limb 0 to possess a SENSCLOSESTANIMX and SENSCLOSESTANIMY sensor.
#define ENFORCESENSORSINTRUNK 0



// SENSORTYPES defines which sensors are to be used in the simulation (besides
// proprioceptors, which are always used).  MODIFY THIS (and also
// NBUSEDSENSORTYPES) if you want to use more or fewer sensors in your
// simulation - of course, don't forget to update fillSensors as well.

//#define NBSENSORTYPES 7
//int SENSORTYPES[NBSENSORTYPES] = {SENSBALLX, SENSBALLY, SENSCLOSESTANIMX,
//                                SENSCLOSESTANIMY, SENSTOUCH, SENSUPDOWN, SENSLEFTRIGHT };

#define NBSENSORTYPES 2
int SENSORTYPES[NBSENSORTYPES] = { SENSUPDOWN, SENSLEFTRIGHT };

// UNCOMMENT THIS (and comment out the previous passage) IF YOU WON'T USE THE
// BALL/BOX IN YOUR SIMULATIONS
//
//#define NBSENSORTYPES 3 int SENSORTYPES[NBSENSORTYPES] = {SENSCLOSESTANIMX,
//SENSCLOSESTANIMY, SENSTOUCH};

int WORLDTYPE = -1;
dReal DAMAGETABLE[REGISTSIZE][REGISTSIZE];
int FREEZE = 0;
int AIRCOLLISIONS = 0;
int BOARD = 0;
int DISABLESENSORS = 0;
int VISUAL = 0;
int NOACTUATE = 0;
int BALL = 0;
int CORRIDOR = 0;
int WALLS = 0;
int OUTPUTREDIRECT = TONULL;
dReal GRAVCORR = 4.0 / (dReal) WORLDRADIUS;
dWorldID world;
dMatrix3 IDENTITY;
dSpaceID globalspace;
dSpaceID alternatespace; // only used within collision method
double upDown = 0.0f;
double leftRight = 0.0f;

// First, a few utility function for displaying messages or saving them to
// disk...

FILE *OUTPUTFILE;

void myprintf(const char *msg, ...)
{
    static va_list argp;
    static int nbcalls=0;
    if (OUTPUTREDIRECT == TONULL) return;
    if (OUTPUTREDIRECT == TOFILE)
    {
        if (nbcalls == 0)
            OUTPUTFILE = fopen("./output", "w");
        if (nbcalls == 50) fflush(OUTPUTFILE);
        if (nbcalls == 10000)
        {
            nbcalls = 1;
            fclose(OUTPUTFILE);
            OUTPUTFILE = fopen("./output", "w");
        }
        va_start(argp, msg);
        vfprintf(OUTPUTFILE, msg, argp);
        va_end(argp);
    }
    else if (OUTPUTREDIRECT == TOSTDOUT)
    {
        va_start(argp, msg);
        vprintf(msg, argp);
        va_end(argp);
    }

}

void myexit(int i)
{
    myprintf("Exiting |\n");
    if (OUTPUTFILE) fflush(OUTPUTFILE);
    exit(i);
}


// exits program, write reason to "exitmessage.txt"
void mydie(const char *msg, ...)
{
    va_list argp;
    FILE *out;
    vprintf(msg, argp);
    fflush(stdout);
    out = fopen("exitmessage.txt", "w");
    va_start(argp, msg);
    vfprintf(out, msg, argp);
    va_end(argp);
    fflush(out);
    myexit(-1);
}


dReal gauss(dReal std)
{
    dReal x, y, expo;
    // There's got to be a better way to do this than doing it
    // iteratively.
    while (1)
    {
        x = (random() % 1000) / 100.0;
        if (random() % 2) x = -x;
        y = (1+random() % 1000) / 100.0;
        expo = (x / std); expo = expo*expo;
        if (y <  (exp (-expo/2.0) / (std * sqrt(2.0 * M_PI))))
            return x;
    }
}

dReal perturbPositive(dReal x)
{
    dReal y;
    y = x + gauss(0.2);
    if (y < 0) y += 1;
    if (y > 1) y -= 1;
    return y;
}

dReal perturb(dReal x)
{
    dReal y;
    y = x + gauss(0.4);
    if (y < 1) y += 2;
    if (y > 1) y -= 2;
    return y;
}

dReal mytanh(dReal x)
{
    return tanh(3.0*x);
}

dReal sigmoid(dReal x)
{
    return 1.0 / (1.0+exp(-6.0*x));
}



int firstocc (int n, int *tab, int size)
{
    int i, found = -1;
    for (i=0; i < size; i++)
        if (tab[i] == n)
        {
            found = i; break;
        }
    return found;
}

dReal invexp(dReal x)
{
    return exp (-2*x*x);
}

static void airCallback(void *data, dGeomID o1, dGeomID o2);
static void nearCallback (void *data, dGeomID o1, dGeomID o2);
//  Still an awful lot of global variables !
dGeomID sphereID; // if sphere world is used
dGeomID groundID; // in case flat world is used
long int tot_time;
int numround;
dContact contact[MAXCONT];	// up to 4 contacts per box-box
dJointGroupID contactgroup;
dGeomID lcorr, rcorr, walln, walls, wallw, walle;
#ifdef DRAWSTUFF
dsFunctions fn;
#endif



class Neuron
{
public:
    int exists; // 0 / 1
    double state; 
    double threshold; // -3/2..3/2
    double out;  // = sigmoid or mytanh(state+threshold)
    int type;
    int function;
    struct{ int exists; int limb; int neur; int rectype; int reftype; double val; 
        json_object* to_json() {
            json_object* bag = json_object_new_object();
            json_object_object_add(bag, "exists", json_object_new_int(exists));
            json_object_object_add(bag, "limb", json_object_new_int(limb));
            json_object_object_add(bag, "neur", json_object_new_int(neur));
            json_object_object_add(bag, "rectype", json_object_new_int(rectype));
            json_object_object_add(bag, "reftype", json_object_new_int(reftype));
            json_object_object_add(bag, "val", json_object_new_double(val));
            return bag;
        }
        void from_json(json_object* bag) {
            exists = json_object_get_int(json_object_object_get(bag, "exists"));
            limb = json_object_get_int(json_object_object_get(bag, "limb"));
            neur = json_object_get_int(json_object_object_get(bag, "neur"));
            rectype = json_object_get_int(json_object_object_get(bag, "rectype"));
            reftype = json_object_get_int(json_object_object_get(bag, "reftype"));
            val = json_object_get_double(json_object_object_get(bag, "val"));
        }
    }
    confrom[MAXCONFROM]; 

    json_object* to_json() {
        json_object* bag = json_object_new_object();
        json_object_object_add(bag, "exists", json_object_new_int(exists));
        json_object_object_add(bag, "state", json_object_new_double(state));
        json_object_object_add(bag, "threshold", json_object_new_double(threshold));
        json_object_object_add(bag, "out", json_object_new_double(out));
        json_object_object_add(bag, "type", json_object_new_int(type));
        json_object_object_add(bag, "function", json_object_new_int(function));
        json_object* array = json_object_new_array();
        for (int i = 0; i < MAXCONFROM; i++) {
            json_object_array_add(array, confrom[i].to_json());
        }
        json_object_object_add(bag, "confrom", array);
        return bag;
    }

    void from_json(json_object* bag) {
        exists = json_object_get_int(json_object_object_get(bag, "exists"));
        state = json_object_get_double(json_object_object_get(bag, "state"));
        threshold = json_object_get_double(json_object_object_get(bag, "threshold"));
        out = json_object_get_double(json_object_object_get(bag, "out"));
        type = json_object_get_int(json_object_object_get(bag, "type"));
        function = json_object_get_int(json_object_object_get(bag, "function"));
        json_object* array = json_object_object_get(bag, "confrom");
        for(int i = 0; i < MAXCONFROM; i++) {
            confrom[i].from_json(json_object_array_get_idx(array, i));
        }
    }

    void randVals()
    {
        threshold = MAXTHRES * (double)(random() % 1000) / 1000.0;
        if (random() % 2) threshold = -threshold;
        /*switch (random() % 3) 
          {
          case 0: 
          function = INVEXP; break;
          case 1: 
          function = SIGMOID; break;
          case 2: 
          function = TANH; break;
          }*/
        //function = SIGMOID;
        switch (random() % 2) 
        {
	    case 0: 
            function = TANH; break;
	    case 1: 
            function = SIGMOID; break;
        }
    }
    Neuron()
    {
        int i;
        type = INTER;
        function = TANH;
        exists = 0;
        for (i=0; i < MAXCONFROM; i++) 
        {
            confrom[i].exists = 0;
            confrom[i].limb = -2; 
            confrom[i].neur= -2; 
            confrom[i].val = 0; 
            confrom[i].rectype = RECSELF;
            confrom[i].reftype = REFBOTH;
        }
        threshold = 0; out = 0; state = 0;
    }
};
    
int isExtSensor(Neuron *N)
{
    for (int i=0; i < NBSENSORTYPES; i++)
        if (N->type == SENSORTYPES[i])
            return 1;
    return 0;
}

class gene
{
    // each gene represents one limb, both morph. and neural info
public:
    int dead;
    int dadnum; // the "dad" gene to this gene
    int terminal;
    int recur; // recursive symmetry (within [0..MAXRECUR] range)
    int symYinputs; // not used
    double lgt;
    double wdt;
    double hgt;
    int symmetrised; // set by the developmental system
    int origgene; // in repres, original num in genome
    double alpha, beta; // angles from ancestor limb
    double alpharec, betarec; // angles for recursion - NOT USED AT PRESENT!
    int orient; // orientation: "vertical" or "horizontal"
    int ref1, ref2, ref3; // bilateral symmetry (only ref1 is used)
    Neuron neurons[MAXNEUR];
    gene()
    {
        dadnum = 0; dead = 0; lgt = 0; wdt=0; hgt=0; alpha=0; beta=0;
        alpharec=0; betarec=0;
        origgene = 0; ref1=0; ref2=0; ref3=0; recur=0; symmetrised=0;
        terminal=0; 
    }
    void randBodyGene(int dad)
    {
        // assigns random values, but only to morphology info (i.e. not neurons
        // info)
        dead = 0;
        dadnum = dad;
        ref1 = 0; ref2 = 0; ref3 = 0;
        lgt = (float)(random() % 1000) / 1000.0;
        wdt = (float)(random() % 1000) / 1000.0;
        hgt = (float)(random() % 1000) / 1000.0;
        alpha = double (random() % 8  - 4) / 8.0; // [-4/8..3/8]
        beta = double (random() % 8  - 4) / 8.0; // [-4/8..3/8]
        //beta = 1.0 - (float)(random() % 2000) / 1000.0;
        // when in recursion, alpha = alpharec / 4
        //		      beta = betarec / 4
        //alpharec = 1.0 - (float)(random() % 2000) / 1000.0;
        //betarec = 1.0 - (float)(random() % 2000) / 1000.0;
        alpharec = double (random() % 8  - 4) / 8.0; // [-4/8..3/8]
        betarec = double (random() % 8  - 4) / 8.0; // [-4/8..3/8]
        symmetrised = 0;
        terminal = random() % 2;
        if (random() % 100 < PROBAINITRECUR) recur = random() % MAXRECUR;
	    else recur=0;
        if (dad == -1) recur = 0;
        if (random() % 100 < PROBAINITREF)
            ref1 = 1; else ref1 = 0;
        if (random()%2) orient = 1; else orient = -1;
    }
    json_object* to_json() {
        json_object* bag = json_object_new_object();
        json_object_object_add(bag, "dead", json_object_new_int(dead));
        json_object_object_add(bag, "dadnum", json_object_new_int(dadnum));
        json_object_object_add(bag, "terminal", json_object_new_int(terminal));
        json_object_object_add(bag, "recur", json_object_new_int(recur));
        json_object_object_add(bag, "symYinputs", json_object_new_int(symYinputs));
        json_object_object_add(bag, "lgt", json_object_new_double(lgt));
        json_object_object_add(bag, "wdt", json_object_new_double(wdt));
        json_object_object_add(bag, "hgt", json_object_new_double(hgt));
        json_object_object_add(bag, "symmetrised", json_object_new_int(symmetrised));
        json_object_object_add(bag, "origgene", json_object_new_int(origgene));
        json_object_object_add(bag, "alpha", json_object_new_double(alpha));
        json_object_object_add(bag, "beta", json_object_new_double(beta));
        json_object_object_add(bag, "alpharec", json_object_new_double(alpharec));
        json_object_object_add(bag, "betarec", json_object_new_double(betarec));
        json_object_object_add(bag, "ref1", json_object_new_int(ref1));
        json_object_object_add(bag, "ref2", json_object_new_int(ref2));
        json_object_object_add(bag, "ref3", json_object_new_int(ref3));
        json_object* array = json_object_new_array();
        for (int i = 0; i < MAXNEUR; i++) {
            json_object_array_add(array, neurons[i].to_json());
        }
        json_object_object_add(bag, "neurons", array);
        return bag;
    }

    void from_json(json_object* bag) {
        dead = json_object_get_int(json_object_object_get(bag, "dead"));
        dadnum = json_object_get_int(json_object_object_get(bag, "dadnum"));
        terminal = json_object_get_int(json_object_object_get(bag, "terminal"));
        recur = json_object_get_int(json_object_object_get(bag, "recur"));
        symYinputs = json_object_get_int(json_object_object_get(bag, "symYinputs"));
        lgt = json_object_get_double(json_object_object_get(bag, "lgt"));
        wdt = json_object_get_double(json_object_object_get(bag, "wdt"));
        hgt = json_object_get_double(json_object_object_get(bag, "hgt"));
        symmetrised = json_object_get_int(json_object_object_get(bag, "symmetrised"));
        origgene = json_object_get_int(json_object_object_get(bag, "origgene"));
        alpha = json_object_get_double(json_object_object_get(bag, "alpha"));
        beta = json_object_get_double(json_object_object_get(bag, "beta"));
        alpharec = json_object_get_double(json_object_object_get(bag, "alpharec"));
        betarec = json_object_get_double(json_object_object_get(bag, "betarec"));
        ref1 = json_object_get_int(json_object_object_get(bag, "ref1"));
        ref2 = json_object_get_int(json_object_object_get(bag, "ref2"));
        ref3 = json_object_get_int(json_object_object_get(bag, "ref3"));
        
        json_object* array = json_object_object_get(bag, "neurons");
        for(int i = 0; i < MAXNEUR; i++) {
            neurons[i].from_json(json_object_array_get_idx(array, i));
        }
    }

};


void ph() { myprintf("Hello/\n"); }

class Ball {
public:
    dBodyID body;
    dJointID joint;
    dGeomID geom;
    dReal sides[3];
    dMass mass;
    int alive;
    void generate()
    {
        sides[0] = 0.5 ;
        sides[1] = 0.5 ;
        sides[2] = 0.5 ;
        BALL = 1;
        dMassSetBox(&mass, 1.0, sides[0], sides[1], sides[2]);
        body = dBodyCreate(world);
        dBodySetMass (body, &mass);
        geom = dCreateBox(globalspace, sides[0], sides[1], sides[2]);
        dGeomSetBody (geom, body);
        initPos();
        alive = 1;
    }
    void remove()
    { 
        dBodyDestroy (body); dGeomDestroy (geom); 
        alive = 0; 
        BALL = 0;
    }
#ifdef DRAWSTUFF
    void draw()
    {
        dsSetColor(0,0,0);
        dsDrawBox (dGeomGetPosition(geom), dGeomGetRotation(geom), sides);
    }
#endif
    void setPos(dReal x, dReal y)
    {
        dMatrix3 mat;
        dBodySetPosition(body, x, y, sides[2]/2.0);
        dBodySetLinearVel(body, 0, 0, 0);
        dBodySetAngularVel(body, 0, 0, 0);
        dRSetIdentity(mat);
        dBodySetRotation(body, mat);
    }
    void initPos()
    {
        dMatrix3 mat;
        dBodySetPosition(body, 0, 0, sides[2]/2.0);
        dBodySetLinearVel(body, 0, 0, 0);
        dBodySetAngularVel(body, 0, 0, 0);
        dRSetIdentity(mat);
        dBodySetRotation(body, mat);
    }
};

class Animat;

class limb {
public:
    dBodyID id;
    dJointID joint;
    dGeomID geom;
    int index;
    dReal lgt;
    int touchesOther;
    dReal oldspeed;
    dReal wdt;
    dReal hgt;
    Animat *owner;
    int immunitytimer;
    dReal alpha, beta;
    dReal damage, damagedone, curdamage;
    dReal sides[3];
    dMatrix3 oldrot;
    dReal oldpos[3];
    dMass mass;
    limb *dad;
    limb() {id = 0; joint=0; oldspeed = 0;}
};
Ball ball;


Animat *regist[REGISTSIZE];

class Animat {
public:
    int alive;
    int symYinputs;
    int idxgen;
    int ID, PID1, PID2; // MUST BE FILLED BY THE PROGRAM
    Animat *closestanimat;
    //dJointGroupID joints;
    dSpaceID space;


// NOTE: both the genome and the repres must be ORDERED and COMPACT.
// Ordered = any gene can only occur in the array BEFORE the genes that
// describe its children limbs
// Compact = a "dead" (i.e. non-existent) gene can NEVER be followed by a
// non-dead gene.

// heritable genetic data
    gene genome[MAXGENES]; 
// actual representation of the creature, post-development
    gene repres[MAXLIMBS]; 
    int age;
    double fitness;

// physica data for each limb, including ODE data structures
    limb limbs[MAXLIMBS]; 

    limb *createLimb (int index, dReal  l, dReal w, dReal h, dReal x, dReal y, 
                      dReal z, dReal alpha);
    limb *addLimb (limb *tr, int idxGen);
//    dReal x, y, z;
    Animat()
    {
        age = 0;
        alive = 0;
        ID = -1;
        fitness = 0;
    }

    json_object* to_json()
    {
        // XXX - only records the genes.
        json_object* array = json_object_new_array();
        for (int i = 0; i < MAXGENES; i++) {
            json_object_array_add(array, genome[i].to_json());
        }
        return array;        
    }

    void from_json(json_object* array)
    {
        for(int i = 0; i < MAXGENES; i++) {
            genome[i].from_json(json_object_array_get_idx(array, i));
        }
    }

    void read(const char *s)
    {
        from_json(json_object_from_file((char *) s));
        /*
          FILE *f = fopen(s, "r");
          fread (this->genome, 1, sizeof(genome), f);
          fflush(f);
          fclose(f);
        */
    }
    void save(const char *s)
    {
        json_object_to_file((char *) s, to_json());
/*
  FILE *f = fopen(s, "w");
  fwrite (this->genome, 1, sizeof(genome), f);
  fflush(f);
  fclose(f);*/
    }

    void saveJson(const char *s)
    {
        json_object_to_file((char *) s, to_json());
    }
    void readOld(const char *s)
    {
        FILE *f = fopen(s, "r");
        fread (this, 1, sizeof(Animat), f);
        fclose(f);
    }
    void saveOld(const char *s)
    {
        FILE *f = fopen(s, "w");
        fwrite (this, 1, sizeof(Animat), f);
        fclose(f);
    }

    int nblimbs()
    {
        int i;
        for (i=0; i < MAXLIMBS; i++)
            if (repres[i].dead) break;
        return i;
    }
    void checkGenome()
    {
        // checks that the genome is valid - obviously incomplete.
        int nbg, i;
        if (genome[0].recur)
        {
            displayGenomeFull();
            mydie("Damn, genome[0] has recur !\n");
        }
        for (i=0; i < MAXGENES-1; i++)
            if (genome[i].dead && !genome[i+1].dead)
            {
                displayGenome();
                mydie("Damn, non-ordered genome !\n");
            }
        for (i=1; i < MAXGENES-1; i++)
        {
            if (genome[i].dead) break;
            if (!genome[i].neurons[0].exists) 
            {
                displayGenome();
                mydie("ERROR ! Actuator does not exist in gene %d!\n", i);
            }
            if (!genome[i].neurons[2].exists) 
            {
                displayGenome();
                mydie("ERROR ! Sensor does not exist in gene %d!\n", i);
            }

        }
        nbg = 0;
        for (i=0; i < MAXGENES-1; i++)
        {
            if (!genome[i].dead) nbg++;
        }
        if (nbg < 2) 
        {
            displayGenome();
            mydie("ERROR ! Only %d gene !\n", nbg);
        }
    }
    int nbGenes()
    {
        int i=0;
        while ((i < MAXGENES) && (!genome[i].dead)) {i++;}
        return i;
    }
    void displayGenome()
    {
        int i, j;
        for (i=0; i < MAXGENES; i++) 
        {
            myprintf("%d-%d ", genome[i].dead, genome[i].dadnum);
            if (! genome[i].dead)
            {
                myprintf("( ");
                for (j=0; j < MAXNEUR; j++)
                {
                    myprintf("ex:%d, ",genome[i].neurons[j].exists);
                    if ( !genome[i].neurons[j].exists) continue;
                    /*for (int k=0; k < MAXCONFROM; k++)
                      myprintf("%d's %d (%.2f) ",
                      genome[i].neurons[j].confrom[k].limb,
                      genome[i].neurons[j].confrom[k].neur,
                      genome[i].neurons[j].confrom[k].val);*/
                    myprintf(", ");
                }
                myprintf(") ");
            }
            myprintf("\n");
        }
        myprintf("\n");
    }
    void displayGenomeFull()
    {
        int i, j;
        for (i=0; i < MAXGENES; i++) 
        {
            myprintf("-Lmb %d: ", i);
            if (genome[i].dead)
                myprintf("dead\n");
            else
            {
                myprintf("(dad:%d, lgt:%.3f, wdt:%.3f, hgt:%.3f, ", 
                         genome[i].dadnum, 
                         genome[i].lgt, genome[i].wdt, genome[i].hgt);
                myprintf("alpha:%.3f, beta:%.3f, alprec:%.3f, betrec:%.3f, rec:%d, ",
                         genome[i].alpha, genome[i].beta, genome[i].alpharec,
                         genome[i].betarec, genome[i].recur);
                myprintf("termi:%d)\n", genome[i].terminal);
                myprintf("( ");
                for (j=0; j < MAXNEUR; j++)
                {
                    myprintf("neur %d: ",j);
                    if ( !genome[i].neurons[j].exists) myprintf("X, ");
                    if ( !genome[i].neurons[j].exists) continue;
                    if (genome[i].neurons[j].type == ACTUATOR)
                        myprintf("actuator, ");
                    if (genome[i].neurons[j].type == SENSTOUCH)
                        myprintf("senshard, ");
                    if (genome[i].neurons[j].type == SENSPROP)
                        myprintf("sensprop, ");
                    if (genome[i].neurons[j].type == SENSBALLX)
                        myprintf("sensballx, ");
                    if (genome[i].neurons[j].type == SENSBALLY)
                        myprintf("sensbally, ");
                    if (genome[i].neurons[j].type == SENSCLOSESTANIMY)
                        myprintf("sensclosestanimy, ");
                    if (genome[i].neurons[j].type == SENSCLOSESTANIMX)
                        myprintf("sensclosestanimx, ");
                    if (genome[i].neurons[j].function == INVEXP)
                        myprintf("invexp, ");
                    if (genome[i].neurons[j].function == SIGMOID)
                        myprintf("sig, ");
                    if (genome[i].neurons[j].function == TANH)
                        myprintf("mytanh, ");
                    myprintf("thres: %.2f, ", 
                             genome[i].neurons[j].threshold);
                    for (int k=0; k < MAXCONFROM; k++)
                    {
                        if (genome[i].neurons[j].confrom[k].exists == 0)
                            myprintf("(o) ");
                        else
                        {
                            myprintf("(%d:%d, %.2f, ",
                                     genome[i].neurons[j].confrom[k].limb,
                                     genome[i].neurons[j].confrom[k].neur,
                                     genome[i].neurons[j].confrom[k].val
                                );
                            if (genome[i].neurons[j].confrom[k].rectype == RECDAD)
                                myprintf("rdad-");
                            if (genome[i].neurons[j].confrom[k].rectype == RECSON)
                                myprintf("rson-");
                            if (genome[i].neurons[j].confrom[k].rectype == RECSELF)
                                myprintf("rself-");
                            if (genome[i].neurons[j].confrom[k].reftype == REFSYMM)
                                myprintf("rsymm)");
                            if (genome[i].neurons[j].confrom[k].reftype == REFORIG)
                                myprintf("rorig)");
                            if (genome[i].neurons[j].confrom[k].reftype == REFBOTH)
                                myprintf("rboth)");
                        }
                    }
                    myprintf(", ");
                }
                myprintf(") ");
            }
            myprintf("\n");
        }
        myprintf("\n");
    }

    void displayRepres()
    {
        int i, j;
        for (i=0; i < MAXLIMBS; i++) 
        {
            myprintf("-Lmb %d: ", i);
            if (repres[i].dead)
                myprintf("dead\n");
            else
            {
                myprintf("(dad:%d, orig:%d, lgt:%.3f, wdt:%.3f, hgt:%.3f, ", 
                         repres[i].dadnum, repres[i].origgene,
                         repres[i].lgt, repres[i].wdt, repres[i].hgt);
                myprintf("alpha:%.3f, beta:%.3f, alprec:%.3f, betrec:%.3f, rec:%d, term:%d) ",
                         repres[i].alpha, repres[i].beta, repres[i].alpharec,
                         repres[i].betarec, repres[i].recur, 
                         repres[i].terminal);
                myprintf("( \n");
                for (j=0; j < MAXNEUR; j++)
                {
                    myprintf("neur %d: ",j);
                    if ( !repres[i].neurons[j].exists) myprintf("X, ");
                    if ( !repres[i].neurons[j].exists) continue;
                    if (repres[i].neurons[j].type == ACTUATOR)
                        myprintf("ACTUATOR, ");
                    if (repres[i].neurons[j].type == SENSTOUCH)
                        myprintf("senshard, ");
                    if (repres[i].neurons[j].type == SENSPROP)
                        myprintf("sensprop, ");
                    if (repres[i].neurons[j].type == SENSBALLX)
                        myprintf("sensballx, ");
                    if (repres[i].neurons[j].type == SENSBALLY)
                        myprintf("sensbally, ");
                    if (repres[i].neurons[j].type == SENSCLOSESTANIMY)
                        myprintf("sensclosestanimy, ");
                    if (repres[i].neurons[j].type == SENSCLOSESTANIMX)
                        myprintf("sensclosestanimx, ");
                    if (repres[i].neurons[j].function == INVEXP)
                        myprintf("invexp, ");
                    if (repres[i].neurons[j].function == SIGMOID)
                        myprintf("sig, ");
                    if (repres[i].neurons[j].function == TANH)
                        myprintf("mytanh, ");
                    myprintf("thres: %.2f, ", 
                             repres[i].neurons[j].threshold);
                    for (int k=0; k < MAXCONFROM; k++)
                    {
                        if (repres[i].neurons[j].confrom[k].exists == 0)
                            myprintf("(o) ");
                        else
                        {
                            myprintf("(%d:%d, %.2f, ",
                                     repres[i].neurons[j].confrom[k].limb,
                                     repres[i].neurons[j].confrom[k].neur,
                                     repres[i].neurons[j].confrom[k].val
                                );
                            if (repres[i].neurons[j].confrom[k].rectype == RECDAD)
                                myprintf("rdad-");
                            if (repres[i].neurons[j].confrom[k].rectype == RECSON)
                                myprintf("rson-");
                            if (repres[i].neurons[j].confrom[k].rectype == RECSELF)
                                myprintf("rself-");
                            if (repres[i].neurons[j].confrom[k].reftype == REFSYMM)
                                myprintf("rsymm)");
                            if (repres[i].neurons[j].confrom[k].reftype == REFORIG)
                                myprintf("rorig)");
                            if (repres[i].neurons[j].confrom[k].reftype == REFBOTH)
                                myprintf("rboth)");
                        }
                    }
                    myprintf(",\n ");
                }
                myprintf(") ");
            }
            myprintf("\n");
        }
        myprintf("\n");
    }

    void compress()
    {
        int to, from, i, k, n;
        for (to=0; to < MAXGENES; to++)
        {
            from = to+1;
            while ((genome[to].dead) && (from < MAXGENES))
            {
                genome[to] = genome[from];
                genome[from].dead = 1;
                // if (!genome[i].dead)  <=== may be useful ??
                for (k=0; k < MAXGENES; k++)
                    if (genome[k].dadnum == from)
                        genome[k].dadnum = to;
                for (n=0; n < MAXGENES; n++)
                    for (i=0; i < MAXNEUR; i++)
                        for (k = 0; k < MAXCONFROM; k++)
                            if (genome[n].neurons[i].confrom[k].limb == from)
                                genome[n].neurons[i].confrom[k].limb = to;
                from++;
            }
            if (from >= MAXGENES) return;
        }
    }


    dReal curDamageFrom(Animat *other)
    {
        // The amount of damage that this other individual is
        // currently dealing to me (i.e. in this very timestep)
        for (int me=0; me < REGISTSIZE; me++)
        {
            if (!regist[me]) continue;
            if (regist[me] != this) continue;
            for (int i=0; i < REGISTSIZE; i++)
            {
                if (!regist[i]) continue;
                if (regist[i] == this) continue;
                if (regist[i] == other)
                    return DAMAGETABLE[i][me];
            }
        }
        return 0;
    }
    
    void addNewGeneOld()
    {
        int i;
        i = nbGenes();
        if (i >= MAXGENES) return;
        genome[i].randBodyGene(random() % i);
        genome[i].dead = 0;
        makeNeuronsLimb(i);
        randConnectionsLimb(i);
    }

    void addNewGene()
    {
        // Adds a new random gene to the genome
        int i, n, k, numdad, numson, num, nbsons, sons[MAXGENES];
        i = nbGenes();
        if (i >= MAXGENES) { myprintf("Can't add new gene, genome full\n");return;}
        numdad = random() % i;
        myprintf("numdad = %d\n", numdad);
        for (num=MAXGENES-2; num > numdad; num--)
        {
            myprintf("pushing %d to %d\n", num, num+1);
            genome[num+1] = genome[num];
            for (k=0; k < MAXGENES; k++)
                if (genome[k].dadnum == num)
                    genome[k].dadnum = num+1;
            for (n=0; n < MAXGENES; n++)
                for (i=0; i < MAXNEUR; i++)
                    for (k = 0; k < MAXCONFROM; k++)
                        if (genome[n].neurons[i].confrom[k].limb == num)
                            genome[n].neurons[i].confrom[k].limb = num+1;
        }
        num = numdad + 1;
        myprintf("num = %d\n", num);
        // temporarily, num must not be a son of numdad!
        genome[num].dadnum = -1; 
        nbsons=0;
        for (i=0; i < MAXGENES; i++)
            if (genome[i].dadnum == numdad)
            {
                sons[nbsons] = i;
                nbsons ++;
            }
        if (nbsons) 
        {
            numson = random() % (nbsons + 1); // we either pick a son, or not
            if (numson < nbsons) // if we do... rebranch that son from num
            {
                myprintf("rebranching %d\n", sons[numson]);
                genome[sons[numson]].dadnum = num;
                for (i=0; i < MAXNEUR; i++)
                    for (k = 0; k < MAXCONFROM; k++)
                        if (genome[sons[numson]].neurons[i].confrom[k].limb 
                            == numdad)
                            genome[sons[numson]].neurons[i].confrom[k].limb 
                                = num;
            }

        }
        genome[num].randBodyGene(numdad);
        genome[num].dead = 0;
        makeNeuronsLimb(num);
        randConnectionsLimb(num);
        reassignBadConns();
    }


    void removeGene(int a)  // DOES NOT COMPRESS THE GENOME !
    {
        // remove a gene from the genome, together with ALL ITS CHILDREN GENES,
        // without compressing the genome
        // GENOME MUST BE COMPRESSED AT SOME POINT AFTER USE
        int i;
        //myprintf("%d.",a);
        for (i=0; i < MAXGENES; i++)
            if ((genome[i].dadnum == a)
                && (!genome[i].dead))
                removeGene(i);
        genome[a].dead = 1;
        for (i=0; i < MAXNEUR; i++)
            genome[a].neurons[i].exists = 0;
    }
   
    void remove1Gene(int a) 
    {
        // removes only this gene, NOT the children genes.
        // This requires reassigning the children genes to the dad of the
        // deleted  gene
        int dad, i, j, k, nbsons=0;
        int tabsons[MAXGENES];
        dad = genome[a].dadnum;
        myprintf("Removing gene %d (dad is %d)\n", a, dad);
        for(i=0; i < MAXGENES; i++)
            if ((genome[i].dadnum == a)
                && (!genome[i].dead))
            {
                tabsons[nbsons] = i;
                nbsons ++;
                genome[i].dadnum = dad;
                for (j=0; j < MAXNEUR; j++)
                    for (k = 0; k < MAXCONFROM; k++)
                        if (genome[i].neurons[j].confrom[k].limb 
                            == a)
                            genome[i].neurons[j].confrom[k].limb 
                                = dad;
            }
        genome[a].dead = 1;
        for (i=0; i < MAXNEUR; i++)
            genome[a].neurons[i].exists = 0;
        for (i=0; i < MAXNEUR; i++)
            if (genome[dad].neurons[i].exists)
                for (j=0; j < MAXCONFROM; j++)
                    if (genome[dad].neurons[i].confrom[j].exists &&
                        (genome[dad].neurons[i].confrom[j].limb == a))
                    {
                        // if conn points from deleted limb, it must be
                        // reassigned. However, I want it to be
                        // reassigned to point from one of the deleted limbs'
                        // sons, not from
                        // the dad or from limb 0 as
                        // randConn might do ...  .. but what if there are no
                        // new sons, i.e. the deleted limb had no child ?
                        if (nbsons)
                        {
                            myprintf("Trying to randomise con %d:%d:%d to a son of %d\n",
                                     dad, i, j, dad);
                            int out=0;
                            do
                            {
                                out=0;
                                randConn(dad, i, j);
                                for (int cpt=0; cpt < nbsons; cpt++)
                                    if (tabsons[cpt] == genome[dad].neurons[i].confrom[j].limb)
                                        out = 1;
                            } while (!out);
			    
                            //while (genome[genome[dad].neurons[i].confrom[j].limb].dadnum 
                            //    != dad ); 
                        }
                        else
                            randConn(dad, i, j);
                    }
        for (i=0; i < MAXNEUR; i++)
            if (genome[0].neurons[i].exists)
                for (j=0; j < MAXCONFROM; j++)
                    if (genome[0].neurons[i].confrom[j].exists &&
                        (genome[0].neurons[i].confrom[j].limb == a))
                        randConn(0, i, j);
        compress();
	
    }

    void insertFromOld(Animat *B, int nfrom, int nto)
    {
        int i = 0, newnum, j, k;
        while ((i < MAXGENES) && (!genome[i].dead)) i++;
        if (i == MAXGENES) return;
        newnum = i;
        genome[newnum] = B->genome[nfrom];
        for (j=0; j < MAXNEUR; j++)
        {
            for (k=0; k < MAXCONFROM; k++)
            {
                if (genome[newnum].neurons[j].confrom[k].limb 
                    == genome[newnum].dadnum)
                    genome[newnum].neurons[j].confrom[k].limb = nto;
                else
                    if (genome[newnum].neurons[j].confrom[k].exists)
                        genome[newnum].neurons[j].confrom[k].limb += 999;
            }
        }
        genome[newnum].dadnum = nto;
        for (i=0; i < MAXGENES; i++)
            for (j=0; j < MAXNEUR; j++)
                for (k=0; k < MAXCONFROM; k++)
                    if (genome[i].neurons[j].confrom[k].exists)
                        if (genome[i].neurons[j].confrom[k].limb - 999 
                            == nfrom)
                            genome[i].neurons[j].confrom[k].limb = newnum;
        for (i=0; i < MAXGENES; i++)
            if ((B->genome[i].dadnum == nfrom) &&
                !(B->genome[i].dead))
                insertFromOld(B, i, newnum);
    }

    void insertFromLessOld(Animat *B, int dadfrom, int dadto)
    {
        int ipoint, insfrom, j, k, l, tab[MAXGENES][2], idx=0;
        for (j=0; j < MAXGENES; j++) { tab[j][0] = -2; }
        for (insfrom=0; insfrom < MAXGENES; insfrom++)
            if (B->genome[insfrom].dadnum == dadfrom)
            {
                // no need to insert deads..
                if (B->genome[insfrom].dead) continue;
                j=0; while (( !genome[j].dead ) && (j <= MAXGENES)) j++;
                if (j == MAXGENES) return;
                ipoint = j;
                genome[ipoint] = B->genome[insfrom];
                tab[idx][0] = insfrom; tab[idx][1] = ipoint; idx++;
                for (j=0; j < MAXNEUR; j++)
                    for (k=0; k < MAXCONFROM; k++)
                    {
                        if (genome[ipoint].neurons[j].confrom[k].limb 
                            == genome[ipoint].dadnum)
                            genome[ipoint].neurons[j].confrom[k].limb = dadto;
                        else
                            if (genome[ipoint].neurons[j].confrom[k].limb != 0)
                                genome[ipoint].neurons[j].confrom[k].limb 
                                    += 999;
                    }
                genome[ipoint].dadnum = dadto;
                for (l=0; l < MAXGENES; l++)
                    for (j=0; j < MAXNEUR; j++)
                        for (k=0; k < MAXCONFROM; k++)
                            if (genome[l].neurons[j].confrom[k].limb -999
                                == insfrom)
                                genome[l].neurons[j].confrom[k].limb  
                                    = ipoint;
            }
        for (insfrom=0; insfrom < MAXGENES; insfrom++)
            if (tab[insfrom][0] != -2) 
                insertFromLessOld(B, tab[insfrom][0], tab[insfrom][1]); 
    }

    void checkConnections()
    {
        // Makes sure everything is OK
        int limb, neur, conn;
        for (limb=0; limb < MAXGENES; limb++)
        {
            if (genome[limb].dead) continue;
            for (neur=0; neur < MAXNEUR; neur++)
            {
                if (!genome[limb].neurons[neur].exists) continue;
                //test:
                for (conn=0; conn < MAXCONFROM; conn++)
                    if (genome[limb].neurons[neur].confrom[conn].exists)
                    {
                        if (genome[limb].neurons[neur].confrom[conn].neur < 0)
                        { 
                            mydie("Damn ! Existing conn to neg neur %d\n",
                                  genome[limb].neurons[neur].confrom[conn].neur); 
                        }
                        if (genome[limb].neurons[neur].confrom[conn].limb < 0)
                        { 
                            mydie("Damn ! Existing conn to neg limb %d!\n", 
                                  genome[limb].neurons[neur].confrom[conn].limb); 
                        }
                    }
            }
        }
    }
    
    void setImmunityTimer(int time)
    {
        for (int i=0; i < nblimbs(); i++)
        {
            limbs[i].immunitytimer = time;
        }
    }
    void resetDamages()
    {
        for (int i=0; i < nblimbs(); i++)
        {
            limbs[i].damage = 0;
            limbs[i].damagedone = 0;
        }
    }
    dReal getTotalDamageDone()
    {
        // total amount of damage I have dealt since last call to
        // resetDamages()
        dReal sum = 0;
        for (int i=0; i < nblimbs(); i++)
            sum += limbs[i].damagedone;
        return sum;
    }
    dReal getTotalDamage()
    {
        // total amount of damage I have received since last call to
        // resetDamages()
        dReal sum = 0;
        for (int i=0; i < nblimbs(); i++)
            sum += limbs[i].damage;
        return sum;
    }
    
    void deleteUnconnNeurons()
    {
        // Not used
        int limb, neur, connect, conn, l2, n2, c2;
        for (limb=0; limb < MAXGENES; limb++)
        {
            if (genome[limb].dead) continue;
            for (neur=0; neur < MAXNEUR; neur++)
            {
                if (!genome[limb].neurons[neur].exists) continue;
                if ((genome[limb].neurons[neur].type == SENSPROP) 
                    || (isExtSensor(&genome[limb].neurons[neur])))
                    continue;
                if (genome[limb].neurons[neur].type == ACTUATOR) continue;
                // if is has a conn from another existing neuron,
                // it is connected
                for (conn=0; conn < MAXCONFROM; conn++)
                    if ((genome[limb].neurons[neur].confrom[conn].exists)
                        && (
                            // hmmm syntax freaks beware..
                            genome[genome[limb].neurons[neur].confrom[conn].limb].neurons[genome[limb].neurons[neur].confrom[conn].neur].exists
                            ))
                        connect = 1;
                // If another neuron has an existing conn from it, it
                // is connected
                for (l2=0; l2 < MAXGENES; l2++)
                {
                    if (genome[l2].dead) continue;
                    for (n2=0; n2 < MAXNEUR; n2++)
                    {
                        if (!genome[l2].neurons[n2].exists) continue;
                        for (c2=0; c2 < MAXCONFROM; c2++)
                            if ((genome[l2].neurons[n2].confrom[c2].exists)
                                && (genome[l2].neurons[n2].confrom[c2].limb == limb)
                                && (genome[l2].neurons[n2].confrom[c2].neur == neur))
                                connect = 1;
                    }
                }
                if (!connect) 
                {
                    myprintf("Unconnected [%d:%d] !\n",limb,neur);
                    genome[limb].neurons[neur].exists = 0;
                }
            }
        }
    }
    
    
    
    void copyFrom(Animat *B)
    {
        copyGenomeOnlyFrom(B);
        ID = B->ID; PID1 = B->PID1; PID2 = B->PID2;
        fitness = B->fitness;
    }
    void copyGenomeOnlyFrom (Animat *B)
    {
        int i;
        for (i=0; i < MAXGENES; i++)
            genome[i] = B->genome[i];
        age = B->age;
    }
    
    void crossWith (Animat *B)
    {
        // Simple crossover method
        // (Methode "bourrin")
        int i, CO1;
        i=0;
        CO1 = 1+ random() % (nbGenes() - 1);
        for (i=CO1; i < MAXGENES; i++)
            genome[i] = B->genome[i];
    }

    void reassignBadConns()
    {
        // Exact contract:
        // any existing conn from an existing limb which is non-adjacent or
        // not root limb OR NOT SAME LIMB, or from a non-existing limb,
        //  is randomly 
        // reassigned to a legitimate source
	
        int li, ne, co, lifrom;
        dReal weight;
        Neuron *tmpn;
        for (li=0; li < MAXGENES; li++)
            for (ne=0; ne < MAXNEUR; ne++)
                for (co=0; co < MAXCONFROM; co++)
                {
                    if (genome[li].dead) continue;
                    tmpn = &genome[li].neurons[ne];
                    if (tmpn->confrom[co].exists)
                    {
                        lifrom = tmpn->confrom[co].limb;
                        weight = tmpn->confrom[co].val;
                        if ( (lifrom != genome[li].dadnum) &&
                             (genome[lifrom].dadnum != li) &&
                             (lifrom != li) &&
                             (li != 0) &&
                             (lifrom != 0))
                        {
                            myprintf("Reassigning %d:%d con %d ", li, ne, co);
                            myprintf("from %d:%d to ", lifrom, tmpn->confrom[co].neur);
                            randConn(li, ne, co);
                            myprintf("%d:%d\n", tmpn->confrom[co].limb, tmpn->confrom[co].neur);
                        }
                        if (genome[lifrom].dead)
                        {
                            myprintf("Reassigning %d:%d con %d ", li, ne, co);
                            myprintf("from %d:%d (%d dead) to ", lifrom, tmpn->confrom[co].neur,
                                     lifrom);
                            randConn(li, ne, co);
                            myprintf("%d:%d\n", tmpn->confrom[co].limb, tmpn->confrom[co].neur);
                        }
                        tmpn->confrom[co].val = weight;
                    }
                }
		    
    }
    
    int insertFrom(Animat *B, int from, int dadto)
    {
        // Inserts a sub-tree from B's limb graph into my own
        // More precisely: insert a copy of limb "from" off animat B (together
        // with all its sub-limbs) to my own limb "dadto"
        // Tricky code.
        int idx, n, i, neu, con, orignewlimb;
        int copyfrom[MAXGENES];
        // the easy part: creating an array to associate limbs to be copied
        // (from B's genome)
        // to places where they should be copied (in my genome). 
        // copyfrom[placeinthegenome] = numoflimbinBtobecopiedhere if a limb
        // is to be copied, -4 otherwise
        idx = 0;
        for (n=0; n < MAXGENES; n++) copyfrom[n] = -4;
        while ((idx < MAXGENES) && (!genome[idx].dead)) idx ++;
        orignewlimb = idx;
        //myprintf("Inserting from orig. idx = %d\n", idx);
        if (idx >= MAXGENES) return -1;
        for (n=0; n < MAXGENES; n++)
        {
            if (B->isInSublimbGenome(n, from))
            {
                //myprintf("In B, %d is in sublimb originating in %d !\n", n, from);
                //myprintf("In B, %d's dad is  %d !\n", n, B->genome[n].dadnum);

                copyfrom[idx] = n;
                do { idx ++ ; } while ((idx < MAXGENES) && (!genome[idx].dead));
                if (idx >= MAXGENES) break;
            }
        }

        // Array made, now the tough part
        for (n=0; n < MAXGENES; n++)
            if (copyfrom[n] != -4)
            {
                //printf("gene %d is copied from B's gene %d\n", n, copyfrom[n]);
                genome[n] = B->genome[copyfrom[n]];

                // should only be -4 for first limb of newly inserted subtree,
                // i.e. :
                if (copyfrom[n] == from)
                    genome[n].dadnum = dadto;
                else
                {
                    for (i=0; i < MAXGENES; i++)
                        if (copyfrom[i] == B->genome[copyfrom[n]].dadnum)
                        {
                            genome[n].dadnum = i; 
                            break;
                        }
                    if (i >= MAXGENES) 
                    {
                        myprintf("Damn ! Couldn't find %d's dad copy\n", n);
                        myprintf("copyfrom[%d] = %d\n",
                                 n, copyfrom[n]);
                        myprintf("Dad of copyfrom[%d] is %d\n", n,
                                 B->genome[copyfrom[n]].dadnum);
                        for (i=0; i < MAXGENES; i++)
                            if (copyfrom[i] == B->genome[copyfrom[n]].dadnum)
                                myprintf("Found cf[%d] = %d\n", i, copyfrom[i]);
                        mydie("i >= MAXGENES\n");
                    }
                }

	
                //modifying neural info...
                for (neu = 0; neu < MAXNEUR; neu++)
                    for (con=0; con < MAXCONFROM; con++)
                        if (genome[n].neurons[neu].confrom[con].exists)
                        {
                            if (genome[n].neurons[neu].confrom[con].limb < 0)
                                mydie("Damn, Existing con from neg limb\n");

                            //myprintf("Treating %d:%d-%d\n", n, neu, con);
			    
                            if (genome[n].neurons[neu].confrom[con].limb
                                == B->genome[from].dadnum)
                                genome[n].neurons[neu].confrom[con].limb = dadto;
                            else
                            {
                                for (i=0; i < MAXGENES; i++)
                                    if (copyfrom[i] ==
                                        genome[n].neurons[neu].confrom[con].limb)
                                        genome[n].neurons[neu].confrom[con].limb = i;
                            }
                        }
            }

        // if the addition results in the new child having the same
        // orientation as a pre-existing child, or has orientation 0,0 in
        // a recursified limb, we try to re-orientate it
        int k = 0;
        int colli = 0;
        do
        {
            colli = 0;
            for (i=0; i < MAXGENES; i++)
            {
                if (((genome[orignewlimb].alpha == 0)
                     && (genome[orignewlimb].beta == 0)
                     && (genome[orignewlimb].recur > 0))
                    ||
                    ((genome[i].dadnum == dadto) 
                     && (genome[i].alpha == genome[orignewlimb].alpha)
                     && (genome[i].beta == genome[orignewlimb].beta)))
                {
                    colli = 1;
                    genome[orignewlimb].alpha = dReal (random() % 8 - 4) / 8.0; // [-4/8..3/8]
                    genome[orignewlimb].beta = dReal (random() % 8 - 4) / 8.0; // [-4/8..3/8]
                }
            }
            k++;
        } while (colli && (k < 100));

// WHAT IF LIMB NOT FULLY GRAFTED ?
        // Well then the conn will be randomly reassigned
        // Note that in our experiments, a creature which reaches the 
        // max number of limbs is discarded, so any creature for
        // which limbs are not fully grafted due to reaching max limit will be
        // discarded as well
        for (i=0; i < MAXGENES; i++)
        {
            if (genome[i].dadnum == -4)
            {
                myprintf("Damn ! Incorrect dad reassignment in grafting !\n");
                myprintf("Error at gene %d with dadnum -4\n", i);
                myprintf("copyfrom[%d] = %d, with dadnum(in B) %d, copyfrom %d\n", 
                         i, copyfrom[i], B->genome[copyfrom[i]].dadnum,
                         copyfrom[B->genome[copyfrom[i]].dadnum]);
                mydie("Incorrect dad reassignment in insertFrom\n");
            }
            if (genome[i].dadnum >= i)
            {
                myprintf("Damn ! Incorrect dad ordering !\n");
                myprintf("Error at gene %d with dadnum %d\n", i, 
                         genome[i].dadnum);
                myprintf("copyfrom[%d] = %d, with dadnum(in B) %d, copyfrom %d\n", 
                         i, copyfrom[i], B->genome[copyfrom[i]].dadnum,
                         copyfrom[B->genome[copyfrom[i]].dadnum]);
                mydie("Incorrect dad ordering in insertFrom\n");
            }

        }
        checkGenome();
        return(orignewlimb);
    }
	    	
   
    void pickFrom(Animat *B)
    {
        int CO1, CO2;
        CO1 = random() % nbGenes();
        CO2 = 1 + random() % (B->nbGenes() - 1);
        myprintf("Picking at CO1 %d CO2 %d\n", CO1, CO2);
        displayGenome();
        B->displayGenome();
        insertFrom (B, CO2, CO1);
    }
	

    
    void graftWith(Animat *B)
    {
        // The other, non-trivial crossover operator.
        // Replace a certain subtree from my limb graph with a sub-tree from
        // the limb graph of B
	
        // How graftWith works: Find a subtree. Delete it. Compress. Insert new 
        // subtree with insertFrom, keeping the same dad as the old, deleted
        // subtree.
        //
        // Together with insertFrom, this is the trickiest part of the code.
        int nb1, nb2, CO1, CO2, insertpoint, i, j, k, found, orignewlimb; 
        Neuron neurtemp[MAXNEUR];  
        nb1 = 1; nb2 = 1; i = 0;
        i=0;
        CO1 = 1+ random() % (nbGenes() - 1);
        found =0;
        CO2 = 1 + random() % (B->nbGenes() - 1); // CO2 must not nec. be a dad !
        myprintf(" CO1: %d, CO2: %d\n", CO1, CO2);
        insertpoint = genome[CO1].dadnum;
        if (insertpoint > CO1) mydie("ERROR ! - Non-ordered genome !\n");
        for (i=0; i < MAXNEUR; i++)
            neurtemp[i] = genome[insertpoint].neurons[i];
        for (j=0; j < MAXNEUR; j++)
            for (k=0; k < MAXCONFROM; k++)
                if (genome[insertpoint].neurons[j].confrom[k].limb == CO1) 
                    genome[i].neurons[j].confrom[k].limb = 997;
        removeGene(CO1);
        compress(); // compress takes care of updating conns referring to limbs moved
        orignewlimb = insertFrom (B, CO2, insertpoint);
        if (orignewlimb == -1)  mydie("Damn ! No place to graft !\n");
        for (i=0; i < MAXGENES; i++)
            for (j=0; j < MAXNEUR; j++)
                for (k=0; k < MAXCONFROM; k++)
                    if (genome[i].neurons[j].confrom[k].limb == 997) 
                        genome[i].neurons[j].confrom[k].limb = orignewlimb;
    }

    
    void clearNeur(int limb, int num)
    {
        // clears up a neuron's info
        int i;
        genome[limb].neurons[num].exists = 0;
        for (i=0; i < MAXCONFROM; i++)
            genome[limb].neurons[num].confrom[i].exists = 0;
    }

    void delNeur(int limb, int num)
    {
        // delete the neuron, AND any connection in the genome that points to
        // it
        int i, j, k;
        for (i=0; i < MAXGENES; i++)
            for (k=0; k < MAXNEUR; k++)
                for (j=0; j < MAXCONFROM; j++)
                    if ((genome[i].neurons[k].confrom[j].limb == limb) 
                        && (genome[i].neurons[k].confrom[j].neur == num))
                        genome[i].neurons[k].confrom[j].exists = 0;
        genome[limb].neurons[num].exists = 0;
        for (i=0; i < MAXCONFROM; i++)
            genome[limb].neurons[num].confrom[i].exists = 0;
    }

		
    void randConn(int limb, int neur, int conn)
    {
        // generate random, valid values for this connection of this neuron of
        // this limb.
        static int tab[MAXGENES*MAXNEUR][2];
        int nbchoice=0;
        int num, i, j;
        //myprintf("in randConn(%d, %d, %d)\n", limb, neur, conn);
        if (genome[limb].dead) return;
        for (i=0; i < MAXGENES; i++)
        {
            if (genome[i].dead) continue; // not dead ! causes infinite loop in rem1gene !
            for (j=0; j < MAXNEUR; j++)
            {
                if (genome[i].neurons[j].exists == 0) continue;
                if (genome[i].neurons[j].type == ACTUATOR) continue; // ?...
                if ((genome[limb].dadnum == i) || (genome[i].dadnum == limb) 
                    || (i == 0) || (limb == 0) // all conns from/to limb 0
                    || ((i == limb) /*&& (j != neur)*/))
		    
                {
                    if (nbchoice >= MAXNEUR * MAXGENES)
                    {
                        mydie("Damn ! Too many choices !\n");
                    }
                    tab[nbchoice][0] = i;  // (i,j) is a valid choice for connection
                    tab[nbchoice][1] = j;  // (i,j) is a valid choice for connection
                    nbchoice++;
                }
            }
        }
        if (nbchoice == 0)
        {
            char str[100];
            sprintf(str,"couldntfindforconn%d:%d:%d.dat", limb, neur, conn);
            save(str);
            // Should never happen if cnx to same limb is authorized (we must 
            // authorize it for animats composed of only limb 0)
            genome[limb].neurons[neur].confrom[conn].exists = 0; 
			mydie("Damn ! Couldn't find any neuron to connect from!\n");
            //myprintf("%d %d Couldn't find any neuron to connect to\n",limb,neur);
            //return;
        }
        num = random() % nbchoice;
        genome[limb].neurons[neur].confrom[conn].limb = tab[num][0];
        genome[limb].neurons[neur].confrom[conn].neur = tab[num][1];
        genome[limb].neurons[neur].confrom[conn].val = (random() % 61) -30;  // -3..3
        genome[limb].neurons[neur].confrom[conn].val /= 30.0;
	
        i = random() % 100;
        if (i < 75) genome[limb].neurons[neur].confrom[conn].reftype = REFBOTH;
        else if (i < 87) genome[limb].neurons[neur].confrom[conn].reftype = REFORIG;
        else genome[limb].neurons[neur].confrom[conn].reftype = REFSYMM;
	
        i = random() % 100;
        if (i < 33) genome[limb].neurons[neur].confrom[conn].rectype = RECSELF;
        else if (i < 66) genome[limb].neurons[neur].confrom[conn].rectype = RECDAD;
        else genome[limb].neurons[neur].confrom[conn].rectype = RECSON;
        /*if (limb == 0)
          printf("Limb 0 neur %d conn %d is from %d:%d\n",
          neur, conn,
          genome[limb].neurons[neur].confrom[conn].limb,
          genome[limb].neurons[neur].confrom[conn].neur);  */
    }


    void mutate()
    {
        // The mutation operator
        int num, i, j;
        Neuron *tmpneur;
        Animat tmpAnimat;
        tmpAnimat.copyFrom(this);
        /*if (random() % 100 < PROBAMUT) 
          for (i=0; i < 10; i++)
          {
          removeGene(1 + random() % (nbGenes() - 1)); 
          compress();
          if (nbGenes() >= MINGENES) break; 
          else copyFrom(&tmpAnimat);
          }*/
        // Adding or deleting genes..
        if (random() % 100 < PROBAMUT) 
            if (nbGenes() > MINGENES)
            {
                for (i=0; i < 10; i++)
                {
                    remove1Gene(1 + random() % (nbGenes() - 1)); 
                    compress();
                    if (nbGenes() >= MINGENES) break; 
                    else copyFrom(&tmpAnimat);
                }
            }
        if (random() % 100 < PROBAMUT)
            addNewGene();
        // Turn one recursive loop in the genome, if any, into separate
        // (identical) genes - source of duplication (and hopefully later
        // exaptation)
        if (random() % 100 < PROBAMUT)
	        dev1RecInGenome();
        //deleteUnconnNeurons();
        checkGenome(); // you never know.
        // Neural mutations...
        for (num=0; num < MAXGENES; num++)
        {
            if (genome[num].dead) break;
            for (i=0; i < MAXNEUR; i++)
            {
                // delete or create neurons
                tmpneur = &genome[num].neurons[i];
                // you can't delete proprioceptors and actuators,
                // or (if sensors are imposed in trunk) the first 2 neurons of
                // limb 0
                if ((random() % 100 < PROBAMUT) 
                    && (tmpneur->type != ACTUATOR)
                    && (tmpneur->type != SENSPROP)
                    && ((!ENFORCESENSORSINTRUNK) || 
                        ((num != 0) || (i > 1))))
                {
                    //myprintf("Flip Neur %d Gene %d..", i, num);
                    /*if ((i != 0) && (i != 1) && (i != 2) && (i != 3) &&
                      (i != 4))*/
                    //((num != 0) || (i != 4)))
                    if (tmpneur->exists) tmpneur->exists = 0;
                    else
                    {
                        tmpneur->exists = 1;
                        tmpneur->randVals();
                        randConnectionsNeur(num, i);
                        if (random () % 100 < PROBANEURSENS)
                        {
                            int val; 
                            val = random() % NBSENSORTYPES;
                            tmpneur->type = SENSORTYPES[val];

                            /*val = random() % 4;
                              if (val == 0) 
                              tmpneur->type = SENSBALLX;
                              if (val == 1) 
                              tmpneur->type = SENSBALLY;
                              if (val == 2) 
                              tmpneur->type = SENSCLOSESTANIMY;
                              if (val == 3) 
                              tmpneur->type = SENSCLOSESTANIMX;*/
                        }
                    }
		    
                }
                // changing neuron data...
                if (tmpneur->exists)
                {
                    // changing neuron type, if allowed...
                    if ((random() % 100 < PROBAMUT) 
                        && (tmpneur->type != ACTUATOR)
                        && (tmpneur->type != SENSPROP)
                        && ((!ENFORCESENSORSINTRUNK) || 
                            ((num != 0) || (i > 1))))
                    {
                        if (isExtSensor(tmpneur))
                        {
                            tmpneur->type = INTER;
                        }
                        else
                        {
                            int val = random() % NBSENSORTYPES;
                            tmpneur->type = SENSORTYPES[val];
                            /*int val; 
                              val = random() % 2;
                              if (val == 0) 
                              tmpneur->type = SENSBALLX;
                              if (val == 1) 
                              tmpneur->type = SENSCLOSESTANIMY;*/
                            /*val = random() % 4;
                              if (val == 0) 
                              tmpneur->type = SENSBALLX;
                              if (val == 1) 
                              tmpneur->type = SENSBALLY;
                              if (val == 2) 
                              tmpneur->type = SENSCLOSESTANIMY;
                              if (val == 3) 
                              tmpneur->type = SENSCLOSESTANIMX;*/
                        }
                    }
                    if (random() % 100 < PROBAMUT)
                    {
                        tmpneur->threshold /= MAXTHRES;
                        tmpneur->threshold = perturb(tmpneur->threshold);
                        tmpneur->threshold *= MAXTHRES;
                    }
                    if (random() % 100 < PROBAMUT) 
                    {
                        switch (random() % 2) 
                        {
                        case 0: tmpneur->function = TANH; break;
                        case 1: tmpneur->function = SIGMOID; break;
                        }
                    }
                    /*if (random() % 100 < PROBAMUT) 
                      {
                      switch (random() % 3) 
                      {
                      case 0: tmpneur->function = INVEXP; break;
                      case 1: tmpneur->function = SIGMOID; break;
                      case 2: tmpneur->function = TANH; break;
                      }
                      }*/
                    for (j=0; j < MAXCONFROM; j++)
                    {
                        if (random() % 100 < PROBAMUT )// / 2) 
                        {
                            if (tmpneur->confrom[j].exists == 0)
                            {
                                tmpneur->confrom[j].exists = 1;
                                randConn(num, i, j);
                            }
                            else
                                tmpneur->confrom[j].exists = 0;
                        }
                        if (tmpneur->confrom[j].exists)
                        {
                            if (random() % 100 < PROBAMUT)
                                randConn(num, i, j);
                            if (random() % 100 < PROBAMUT)
                                tmpneur->confrom[j].val = 
                                    perturb(tmpneur->confrom[j].val);
                            if (random() % 100 < PROBAMUT)
                            {
                                int tmp;
                                tmp = random() % 3;
                                if (tmp == 0) 
                                    tmpneur->confrom[j].reftype = REFBOTH;
                                else if (tmp == 1) 
                                    tmpneur->confrom[j].reftype = REFORIG;
                                else 
                                    tmpneur->confrom[j].reftype = REFSYMM;
                            }
                            if (random() % 100 < PROBAMUT)
                            {
                                int tmp;
                                tmp = random() % 3;
                                if (tmp == 0) 
                                    tmpneur->confrom[j].rectype = RECSELF;
                                else if (tmp == 1) 
                                    tmpneur->confrom[j].rectype = RECDAD;
                                else 
                                    tmpneur->confrom[j].rectype = RECSON;
                            }
                        }

                    }

                }
            }
            // morphological mutation
            for (i=0; i < 12; i++)
            {
                if (random() % 100 < PROBAMUT) 
                    switch (i)
                    {
                    case 0:
                        if (num != 0) genome[num].dadnum = random() % num;
                        break;
                    case 1:
                        myprintf("old lgt %f",genome[num].lgt);
                        genome[num].lgt = 
                            perturbPositive(genome[num].lgt);
                        myprintf(", new lgt %f\n",genome[num].lgt);
                        break;
                    case 2:
                        myprintf("old wdt %f",genome[num].lgt);
                        genome[num].wdt = 
                            perturbPositive(genome[num].wdt);
                        myprintf(", new wdt %f\n",genome[num].lgt);
                        break;
                    case 3:
                        genome[num].hgt = 
                            perturbPositive(genome[num].hgt);
                        break;
                    case 4:
                        myprintf("Old alpha: %f ", genome[num].alpha);
                        genome[num].alpha *= 8;
                        genome[num].alpha += (random() % 5 - 2);
                        if (genome[num].alpha > 3)
                            genome[num].alpha -= 8;
                        if (genome[num].alpha < -4)
                            genome[num].alpha += 8;
                        genome[num].alpha = genome[num].alpha / 8.0;
                        //genome[num].alpha = 
                        //perturb(genome[num].alpha);
                        myprintf("New alpha: %f\n", genome[num].alpha);
                        break;
                    case 5:
                        myprintf("Old beta: %f ", genome[num].beta);
                        genome[num].beta *= 8;
                        genome[num].beta += (random() % 5 - 2);
                        if (genome[num].beta > 3)
                            genome[num].beta -= 8;
                        if (genome[num].beta < -4)
                            genome[num].beta += 8;
                        genome[num].beta = genome[num].beta / 8.0;
                        //perturb(genome[num].beta);
                        myprintf("New beta: %f\n", genome[num].beta);
                        break;
                    case 6:
                        if (random()%2) 
                            genome[num].orient = 1; 
                        else genome[num].orient = -1;
                        break;
                    case 7:
                        if (genome[num].ref1) 
                            genome[num].ref1 = 0; 
                        else 
                        {
                            genome[num].ref1 = 1; 
                            // if we are symmetrising a limb, but this limb is in the
                            // vertical plane, we would want to "angle" it a bit so the
                            // two symmetric copies are not "melted" together
                            if (genome[num].alpha == 0) 
                            {
                                if (random() % 2)
                                    genome[num].alpha--;
                                else
                                    genome[num].alpha++;
                            }
                            if (genome[num].alpha == -4) 
                            {
                                if (random() % 2)
                                    genome[num].alpha++;
                                else
                                    genome[num].alpha = 3;
                            }
                            if ((genome[num].beta == 2) || (genome[num].beta == -2))
                            {
                                if (random() % 2)
                                    genome[num].beta--;
                                else
                                    genome[num].beta++;
                            }
                        }
                        break;
                    case 8:
                        if (genome[num].recur)
                            genome[num].recur = 0;
                        else
                            if (num != 0)
                                genome[num].recur = 1+ random() % (MAXRECUR-1);
                        break;
                    case 9:
                        genome[num].terminal = 1-genome[num].terminal;
                        break;
                    case 10:
                        myprintf("Old alpharec: %f ", genome[num].alpharec);
                        genome[num].alpharec = (dReal)(random() % 8 - 4) / 8.0;
                        //genome[num].alpharec = 
                        //perturb(genome[num].alpharec);
                        myprintf("New alpharec: %f\n", genome[num].alpharec);
                        break;
                    case 11:
                        myprintf("Old betarec: %f ", genome[num].beta);
                        genome[num].betarec = (dReal)(random() % 8 - 4) / 8.0;
                        //genome[num].betarec = 
                        //perturb(genome[num].betarec);
                        myprintf("New betarec: %f\n", genome[num].betarec);
                        break;
                    }
            }
        }
    }

    void findClosestAnimat()
    {
        // find other animat currently alive whoe trunk (= root limb = limb 0)
        // is closest to my trunk
        dReal bestdist = 999999, dist;
        dReal *posother;
        dVector3 vectother;
        for (int n=0; n < REGISTSIZE; n++)
        {
            if (!regist[n]) continue;
            if (regist[n] == this) continue;
            posother= (dReal*) 
                dBodyGetPosition(regist[n]->limbs[0].id);
            dBodyGetPosRelPoint(limbs[0].id, posother[0], 
                                posother[1], posother[2], vectother);
            dist = vectother[0]*vectother[0] 
                + vectother[1]*vectother[1]
                + vectother[2]*vectother[2];
            if (dist < bestdist)
            {
                bestdist = dist;
                closestanimat = regist[n];
            }
        }
    }

    void fillSensors();
    
    void actuate()
    {
        // neural actuation cycle: get sensor values, then update all neuron's
        // states and outputs
        int n, i, j, run; dReal speed, value;
        Neuron *tmpneur;
        speed=0;
        // get sensor data...
        fillSensors();
        //fillSensorsDist();
        for (run=0; run < NBNEURCYCLES; run++)
        {
            for (n=0; n < MAXLIMBS; n++)
            {
                if (repres[n].dead) continue;
                for (i=0; i < MAXNEUR; i++)
                {
                    tmpneur = &repres[n].neurons[i];
                    if (!tmpneur->exists) continue;
                    // don't touch input neurons ! 
                    if ((tmpneur->type == SENSPROP) 
                        || (isExtSensor(tmpneur)))
                        continue;
                    tmpneur->state = 0; 
                    for (j=0; j < MAXCONFROM; j++)
                        if (tmpneur->confrom[j].exists)
                        {
                            if (repres[tmpneur->confrom[j].limb].dead)
                            {
                                mydie("Error: connection from dead limb\n");
                            }
                            if (repres[tmpneur->confrom[j].limb].
                                neurons[tmpneur->confrom[j].neur].exists)
                            {
                                value = 0;
                                // If I'm ref'ed and conn is from my phys dad..
                                // AND if i'm not in a recursion...
                                if (genome[ repres[n].origgene ].ref1
                                    && (tmpneur->confrom[j].limb
                                        == repres[n].dadnum)
                                    && (repres[n].origgene !=
                                        repres[tmpneur->confrom[j].limb].origgene))
                                {
                                    if (tmpneur->confrom[j].reftype==REFBOTH)
                                        // both neurons can receive dad's
                                        // input
                                        value = 
                                            tmpneur->confrom[j].val * 
                                            repres[tmpneur->confrom[j].limb].
                                            neurons[tmpneur->confrom[j].neur].out;
                                    else
                                    {
                                        int mysym;
                                        for (mysym=0; mysym< MAXLIMBS; mysym++)
                                            if ((repres[mysym].origgene ==
                                                 repres[n].origgene) &&
                                                (repres[mysym].dadnum ==
                                                 repres[n].dadnum) &&
                                                (n != mysym) &&
                                                !repres[mysym].dead)
                                                break;
                                        if (mysym == MAXLIMBS)
                                        {
                                            mydie("Error: couldn't find symmetric of limb %d\n!", n);
                                        }
                                        if (((n < mysym) && (tmpneur->confrom[j].reftype == REFORIG))
                                            || ((n > mysym) && (tmpneur->confrom[j].reftype == REFSYMM)))
                                            value =
                                                tmpneur->confrom[j].val *
                                                repres[tmpneur->confrom[j].limb].
                                                neurons[tmpneur->confrom[j].neur].out;
                                        else
                                            value = 0;

                                    }

                                }
                                else if ((repres[tmpneur->confrom[j].limb].dadnum == n)
                                         && genome[ repres[tmpneur->confrom[j].limb].origgene ].ref1
                                         && (repres[tmpneur->confrom[j].limb].origgene
                                             != repres[n].origgene))
                                    // if conn is from my son and this son is
                                    // reflected (i.e. duplicated through
                                    // bilateral symmetry)..
                                    // AND this is the original son (not the
                                    // reflected/symmetrical version)...
                                    // then I must find out how to deal with
                                    // the input (i.e. take it from the
                                    // original son, the reflected/symmetrical
                                    // son, or averaging the 2)
                                {
                                    int zesym, zeson, tmp;
                                    zeson = tmpneur->confrom[j].limb;
                                    // finding the reflected copy of this son
                                    for (zesym=0; zesym < MAXLIMBS; zesym++)
                                        if ((repres[zesym].origgene ==
                                             repres[zeson].origgene) &&
                                            (repres[zesym].dadnum ==
                                             repres[zeson].dadnum) &&
                                            (zeson != zesym) &&
                                            !repres[zesym].dead)
                                            break;
                                    if (zesym == MAXLIMBS)
                                    {
                                        mydie("Error 2: couldn't find symmetric of limb %d\n!", 
                                              zeson);
                                    }
                                    // the lowest number is the original, the
                                    // higher number is
                                    // the symm
                                    if (zesym < zeson) { tmp=zeson; zeson=zesym; zesym=tmp; }
                                    if (tmpneur->confrom[j].reftype == REFORIG)
                                        value =
                                            tmpneur->confrom[j].val *
                                            repres[zeson].neurons[tmpneur->confrom[j].neur].out;
                                    else if (tmpneur->confrom[j].reftype == REFSYMM)
                                        value =
                                            tmpneur->confrom[j].val *
                                            repres[zesym].neurons[tmpneur->confrom[j].neur].out;
                                    else if (tmpneur->confrom[j].reftype == REFBOTH)
                                        value =
                                            tmpneur->confrom[j].val *
                                            (repres[zesym].neurons[tmpneur->confrom[j].neur].out
                                             +repres[zeson].neurons[tmpneur->confrom[j].neur].out)
                                            /2.0;
                                }
                                else
                                {
                                    value = 
                                        tmpneur->confrom[j].val * 
                                        repres[tmpneur->confrom[j].limb].
                                        neurons[tmpneur->confrom[j].neur].out;
                                    /*myprintf("%.4f l%d n%d c%d from l%d n%d\n", 
                                      value, n, i, j, 
                                      tmpneur->confrom[j].limb,
                                      tmpneur->confrom[j].neur
                                      );*/
                                }
                                tmpneur->state += value;
                                /*if ((n == 1) && (i == 1) && (j == 0))
                                  myprintf("neur 1:1 receives %.3f from conn 0\n", value);*/
                            }
                        }
                }
            }

            // neuron's internal states have been updated from inputs. Now
            // calculating outputs from those states...
            for (n=0; n < MAXLIMBS; n++)
            {
                if (repres[n].dead) continue;
                for (i=0; i < MAXNEUR; i++)
                {
                    tmpneur = &repres[n].neurons[i];
                    // again, don't touch sensor neurons (their output is
                    // already filled by fillSensors()
                    if ((tmpneur->type == SENSPROP) 
                        || (isExtSensor(tmpneur)))
                        continue;
                    if (tmpneur->exists)
                    {
                        int c, nbconns;
                        nbconns = 0;
                        for (c=0; c < MAXCONFROM; c++)
                            if (tmpneur->confrom[c].exists
                                && !repres[tmpneur->confrom[c].limb].dead // shouldn't happen
                                && repres[tmpneur->confrom[c].limb].neurons[tmpneur->confrom[c].neur].exists)
                                nbconns++;


                        if ((nbconns == 0) && (tmpneur->state != 0))
                        {
                            mydie("Damn ! receives input without conns ! State:%f\n",
                                  tmpneur->state);
                        }


                        if (nbconns == 0)
                            tmpneur->state = 0;
                        else
                            tmpneur->state = tmpneur->state / (dReal)nbconns;


                        if (tmpneur->type == ACTUATOR)
                        {
                            //myprintf("%.3f\n", tmpneur->state);
                            tmpneur->out = 
                                mytanh(tmpneur->state);
                            //mytanh(tmpneur->state + tmpneur->threshold);
                            //tmpneur->state + tmpneur->threshold;
                        }
                        else
                        {
                            if (tmpneur->function == SIGMOID)
                                tmpneur->out = 
                                    sigmoid(tmpneur->state + tmpneur->threshold);
                            /*else if (tmpneur->function == INVEXP)
                              tmpneur->out = 
                              invexp(tmpneur->state + tmpneur->threshold);*/
                            else if (tmpneur->function == TANH)
                                tmpneur->out = 
                                    mytanh(tmpneur->state + tmpneur->threshold); 
                            else 
                                mydie("Error : Unknown output function!\n");
                        }
				
                    }
                    //   tmpneur->out = 1;
                    else
                        tmpneur->out = 0;


                }
            }
        }

        /*myprintf("Neur 1:3 has out %.4f\n", repres[1].neurons[3].out);
          myprintf("Neur 2:3 has out %.4f\n", repres[2].neurons[3].out);
          myprintf("Neur 0:3 has state %.4f\n", repres[0].neurons[3].state);*/
	
        for (i=1; i < nblimbs(); i++)
        {
            if (NOACTUATE)
            {
                dJointSetHingeParam(limbs[i].joint, dParamFMax, 0);
            }
            else
            {
                dJointSetHingeParam(limbs[i].joint, dParamFMax, MAXFORCE);
                if (FREEZE) speed = 0;
                else
                {
                    // damping...
                    speed = 2.0 * limbs[i].oldspeed + SPEEDMULT *
                        repres[i].neurons[0].out ;
                    speed = speed / 3.0;
                    limbs[i].oldspeed = speed;
                }
                dJointSetHingeParam(limbs[i].joint, dParamVel, speed);

                /*if (VISUAL) 
                  myprintf("Limb %d : Act %.3f, Sen %f, Angle %f, Rate %f\n", 
                  i, repres[i].neurons[0].out,
                  repres[i].neurons[2].out, 
                  dJointGetUniversalAngle1(limbs[i].joint),
                  0.0);
                  //dJointGetUniversalAngleRate1 (limbs[i].joint));*/
            }
        }
    }

    void makeNeuronsLimb(int limb)
    {
        // generates random, valid neural information for this limb (i.e. gives
        // an actuator and a proprioceptor, etc.)
        int neur;
        //myprintf("Making neurs for limb %d\n", limb);
        for (neur=0; neur < MAXNEUR; neur++)
        {
            genome[limb].neurons[neur].exists = 0;
            if ((neur == 0) && (limb == 0) && (ENFORCESENSORSINTRUNK))
            {
                genome[limb].neurons[neur].type = SENSCLOSESTANIMX;
                genome[limb].neurons[neur].exists = 1;
                genome[limb].neurons[neur].randVals();
            }
            else if ((neur == 1) && (limb == 0) && (ENFORCESENSORSINTRUNK))
            {
                genome[limb].neurons[neur].type = SENSCLOSESTANIMY;
                genome[limb].neurons[neur].exists = 1;
                genome[limb].neurons[neur].randVals();
            }
            else if ((neur == 0) && (limb != 0))
            {
                genome[limb].neurons[neur].type = ACTUATOR;
                genome[limb].neurons[neur].exists = 1;
                genome[limb].neurons[neur].randVals();
            }
            //else if (((neur == 3) && (limb == 0)) || 
            else if ((neur == 2) && (limb != 0))
            {
                genome[limb].neurons[neur].type = SENSPROP;
                genome[limb].neurons[neur].exists = 1;
                genome[limb].neurons[neur].randVals();
            }
            else if (random() % 100 < PROBANEURSENS)
            {
                int val = random() % NBSENSORTYPES;
                genome[limb].neurons[neur].type = SENSORTYPES[val];
                genome[limb].neurons[neur].exists = 1;
                genome[limb].neurons[neur].randVals();
            }
            else 
            {
                genome[limb].neurons[neur].type = INTER;
                if (random() % 100 < PROBANEUREXISTS)
                {
                    genome[limb].neurons[neur].exists = 1;
                    genome[limb].neurons[neur].randVals();
                }
            }
        }
    }

    void randConnectionsLimb(int limb)
    {
        // randomise neural connections within this limb
        int neur;
        if (genome[limb].dead) return;
        for (neur=0; neur < MAXNEUR; neur++)
        {
            if ( ! genome[limb].neurons[neur].exists) continue;
            randConnectionsNeur(limb, neur);
        }
    }

    void randConnectionsNeur(int limb, int neur)
    {
        // randomise neural connections within this neuron
        int conn;
        for (conn=0; conn < MAXCONFROM; conn++)
        {
            if (random() % 100 > PROBACONNEXISTS) 
            {
                genome[limb].neurons[neur].confrom[conn].exists = 0;
            }
            else 
            {
                genome[limb].neurons[neur].confrom[conn].exists = 1;
                randConn(limb, neur, conn);
            }
        }
    }
	    

    void printRepres()
    {
        int i;
        for (i=0; i < MAXLIMBS; i++)
            myprintf("%d", repres[i].dead);
        myprintf("\n");
    }
    void printGenome()
    {
        int i;
        for (i=0; i < MAXGENES; i++)
            myprintf("%d", genome[i].dead);
        myprintf("\n");
    }
    void printGenomeFull()
    {
        int i;
        for (i=0; i < MAXGENES; i++)
        {
            if (genome[i].dead)
                myprintf("1 ");
            else
                myprintf("0 (%f, %f, %f) ",
                         genome[i].lgt, genome[i].wdt, genome[i].hgt);
        }
        myprintf("\n");
    }
    void readRepres(dReal x, dReal y, dReal z, dReal alpha);

    void randGenome()
    {
        // creates a random, valid genome. Does NOT test for internal
        // collisions
	
        int idx, nbg;
        int i,  mydad;
        //int j, nbsons;
        genome[0].dead = 0;
        for (i=1; i < MAXGENES; i++) genome[i].dead = 1;
        genome[0].randBodyGene(-1);
        nbg = MINGENES + random() % (MAXGENES - MINGENES);
        for (idx=1; idx < nbg; idx++)
        {
            // Only choose a dad if it has max. 2 other sons
            //  => 4 links max (3 sons, 1 dad)
            /*do 
              {*/
            mydad = random() % idx;
            /*nbsons = 0;
              for (j=0; j < idx; j++)
              if (genome[j].dadnum == mydad)
              nbsons ++;
              } while (nbsons > 2);*/
            genome[idx].randBodyGene(mydad);
            genome[idx].dead = 0;
        }
        if (idx < MINGENES-1) {mydie("Damn! Not enough genes!"); }
        for (i=0; i < MAXGENES; i++)
            makeNeuronsLimb(i);
        for (i=0; i < MAXGENES; i++)
            randConnectionsLimb(i);
        age = 0;
        fitness = 0;
        checkGenome();
    }


    // Note: all "push.." functions must be called while the animat is alive
    // (i.e. after it's been generated) - obviously.
    
    void pushBeforeXVert(dReal X)
    {
        // push the indiv on the x+ side of a horizontal line
        dReal *trunkpos;
        dReal aabb[6];
	
        trunkpos = (dReal* )dBodyGetPosition(limbs[0].id);
        dGeomGetAABB ((dGeomID)space, aabb);
        setPos(X + (trunkpos[0] - aabb[0]), trunkpos[1]);
    }
    
    void pushBehindXVert(dReal X)
    {
        // push the indiv on the x- side of a horizontal line
        dReal *trunkpos;
        dReal aabb[6];
	
        trunkpos = (dReal* )dBodyGetPosition(limbs[0].id);
        dGeomGetAABB ((dGeomID)space, aabb);
        setPos(X - (aabb[1] - trunkpos[0]), trunkpos[1]);
    }
    
    void pushBehindX(dReal X)
    {
        // push the indiv on the x- side of a slanted plane (slanted 45 deg.
        // towards negative x), as in Sims' "Coevolving artificial creatures"
        // paper

        int touches;
        dVector3 sides = {.1,20,20}; // vertical 'slice'
        dVector3 centre = {X, 0, 0};
        dMatrix3 rot;
        dVector3 limbsides;
        dReal *trunkpos;
        dReal xshift;
        dReal aabb[6];
	
        trunkpos = (dReal* )dBodyGetPosition(limbs[0].id);
        dGeomGetAABB ((dGeomID)space, aabb);
        xshift = aabb[5]-aabb[4] + aabb[1]-trunkpos[0]; // push by x+z
        setPos(X-xshift, trunkpos[1]);
	
        dRFromAxisAndAngle(rot, 0, 1, 0, -(M_PI / 4.0)); // slant back 45deg
        do
        {
            touches = 0;
            for (int i=0; i < MAXLIMBS; i++)
                if (limbs[i].id)
                {
                    dGeomBoxGetLengths(limbs[i].geom, limbsides);
                    touches = dBoxTouchesBox(centre, rot, sides,
                                             dBodyGetPosition(limbs[i].id), 
                                             dBodyGetRotation(limbs[i].id),
                                             limbsides
                        );
                    if (touches) break;
                }
            if (!touches)
            {
                setPos(dBodyGetPosition(limbs[0].id)[0] + 0.1,
                       dBodyGetPosition(limbs[0].id)[1]);
                myprintf("not touching!\n");
            }
        } while (!touches);
        myprintf("touching!\n");
    }
    
    void pushBeforeX(dReal X)
    {
        // push the indiv on the x+ side of a slanted plane (slanted 45 deg.
        // towards positive x), as in Sims' "Coevolving artificial creatures"
        // paper
        int touches;
        dVector3 sides = {.1,20,20}; // vertical 'slice'
        dVector3 centre = {X, 0, 0};
        dMatrix3 rot;
        dVector3 limbsides;
        dReal *trunkpos;
        dReal xshift;
        dReal aabb[6];
	
        trunkpos = (dReal* )dBodyGetPosition(limbs[0].id);
        dGeomGetAABB ((dGeomID)space, aabb);
        xshift = aabb[5]-aabb[4] + trunkpos[0]-aabb[0]; // push by x+z
        setPos(X+xshift, trunkpos[1]);
	
        dRFromAxisAndAngle(rot, 0, 1, 0, (M_PI / 4.0)); // slant forth 45deg
        do
        {
            touches = 0;
            for (int i=0; i < MAXLIMBS; i++)
                if (limbs[i].id)
                {
                    dGeomBoxGetLengths(limbs[i].geom, limbsides);
                    touches = dBoxTouchesBox(centre, rot, sides,
                                             dBodyGetPosition(limbs[i].id), 
                                             dBodyGetRotation(limbs[i].id),
                                             limbsides
                        );
                    if (touches) break;
                }
            if (!touches)
            {
                setPos(dBodyGetPosition(limbs[0].id)[0] - 0.1,
                       dBodyGetPosition(limbs[0].id)[1]);
                myprintf("not touching!\n");
            }
        } while (!touches);
        myprintf("touching!\n");
    }


    int isInSublimbGenome(int limb, int anc) 
    {
        // to be used for the genome only
        // Is gene "anc" part of "limb"'s sub-tree (i.e. children limbs + their
        // own children, etc.) ?
        int num; 
        //myprintf("In isInSubLimb\n");
        if (genome[limb].dead) return 0;
        //displayGenome();
        num = limb;
        if (num == anc) return 1;
        do { 
            //myprintf("going from %d to its dad %d, ",num, genome[num].dadnum);
            if (num == genome[num].dadnum)
            {
                myprintf("Damn, %d has dad %d !\n", num, genome[num].dadnum);
                exit(0);
            }
            num = genome[num].dadnum; 
        } while ((num != -1) && (num != anc));
        //if (num == anc) myprintf(" %d == ancestor sought %d\n", num, anc);
        //	 else myprintf("%d is not part of %d sublimb\n", limb, anc);
        if (num == anc) return 1; else return 0;
	
    }

    int isInSublimbRepres(int limb, int anc) 
    {
        // to be used for the repres, that is, the developed genome that
        // contains all limbs (including duplicates, symmetrics, etc.)
        // Is limb "anc" part of "limb"'s sub-tree (i.e. children limbs + their
        // own children, etc.) ?
        int num; 
        if (repres[limb].dead) return 0;
        num = limb;
        if (num == anc) return 1;
        do { 
            num = repres[num].dadnum; 
        } while ((num != -1) && (num != anc));
        if (num == anc) return 1; else return 0;
	
    }
   
    void transGenome()
    {
        // Reads the genome into the repres, taking into account the
        // developmental information (recursivity, symmetry, etc.)
        int i;
        /*transGenome(0, -1, 1, 1, 0);
          myprintf("\n");*/
        for (i=0; i < MAXLIMBS; i++)
        {
            repres[i].dead=1;
            repres[i].symmetrised = 0;
        }
        transGeneSeg(0, -1, 0, 0);
        transNeurons();
    }

    void transGeneSeg(int zegene, int zedad, int numrec, int symm)
    {
        // Does the real work of the previous function, but does not correctly
        // update neurons (transNeurons() does that) .

        // Note: This is a recursive function (as suits the tree structure of
        // genomes) 
	
        int limbnum, i;
        //myprintf("transGeneSeg(%d, %d, %d, %d)\n",
        //	zegene, zedad, numrec, symm);
        limbnum = nblimbs();
        if (limbnum >= MAXLIMBS) return;
        repres[limbnum] = genome[zegene];
        repres[limbnum].dead = 0;
        repres[limbnum].origgene = zegene;
        repres[limbnum].dadnum = zedad;
        if (symm)
            repres[limbnum].symmetrised = 1-repres[zedad].symmetrised;
        else
            repres[limbnum].symmetrised = repres[zedad].symmetrised;
        if (repres[limbnum].symmetrised) 
            repres[limbnum].alpha *= -1;
        if (repres[zedad].origgene == zegene) // recursion !
        {
            //repres[limbnum].alpha = repres[limbnum].alpharec / 4.0;
            //repres[limbnum].beta = repres[limbnum].betarec / 4.0;
            repres[limbnum].alpha = 0;
            repres[limbnum].beta = 0;
            if (repres[limbnum].symmetrised) 
                repres[limbnum].alpha *= -1;
        }
        for (i=0; i < MAXGENES; i++)
            if ((genome[i].dadnum == zegene) && !genome[i].terminal 
                && !genome[i].dead)
            {
                transGeneSeg(i, limbnum,  genome[i].recur, 0);
                if (genome[i].ref1)
                    transGeneSeg(i, limbnum,  genome[i].recur, 1);
            }
        if (numrec && (limbnum != 0)) // LIMB 0 CANNOT BE RECURRED (control centre)
        {
            //myprintf("Recur in gene %d\n", zegene);
            transGeneSeg(zegene, limbnum, numrec -1, 0);
            limbnum = nblimbs()-1;
        }
        for (i=0; i < MAXGENES; i++)
            if ((genome[i].dadnum == zegene) && genome[i].terminal
                && !genome[i].dead && !numrec)
            {
                transGeneSeg(i, limbnum,  genome[i].recur, 0);
                if (genome[i].ref1)
                    transGeneSeg(i, limbnum,  genome[i].recur, 1);
            }

    }

    void transNeurons()
    {
        // develops the neural information from the genome into the repres,
        // taking into account developmental info (i.e. recursive segmentation,
        // symmety, etc.)

        // Tricky code.
	
        int limbnum, neur, conn, i, found;
        Neuron *tmpneur;
        for (limbnum=0; limbnum < MAXLIMBS; limbnum++)
        {
            if (repres[limbnum].dead) break;
            for (neur=0; neur < MAXNEUR; neur++)
            {
                tmpneur = &repres[limbnum].neurons[neur];
                if (!tmpneur->exists) continue;
                for(conn=0; conn < MAXCONFROM; conn++)
                {
                    found = 0;
                    if (!tmpneur->confrom[conn].exists) continue;
                    if (tmpneur->confrom[conn].limb == repres[limbnum].origgene)
                    {
                        // if conn comes from my own origgene.. then we must
                        // check
                        // whether I'm being recursively duplicated, and if so,
                        // check the genetic info to know how to deal with the
                        // connection
                        int isinrec = 0;
                        if (repres[limbnum].dadnum != -1) 
                        {
                            if (repres[limbnum].origgene == 
                                repres[repres[limbnum].dadnum].origgene)
                            {
                                isinrec = 1;
                                if (tmpneur->confrom[conn].rectype == RECDAD)
                                {
                                    tmpneur->confrom[conn].limb=
                                        repres[limbnum].dadnum;
                                    found = 1;
                                }
                            }
                        }
                        for (i=0; i < MAXLIMBS; i++)
                        {
                            if (repres[i].dead) break;
                            if ((repres[i].dadnum == limbnum) &&
                                (repres[i].origgene==repres[limbnum].origgene))
                            {
                                isinrec = 1;
                                if (tmpneur->confrom[conn].rectype == 
                                    RECSON)
                                {
                                    tmpneur->confrom[conn].limb = i;
                                    found = 1;
                                }
                            }
                        }
                        if (isinrec && (!found))
                        {
                            if (tmpneur->confrom[conn].rectype == RECSELF)
                                tmpneur->confrom[conn].limb = limbnum;
                            else
                                tmpneur->confrom[conn].exists = 0;
                            found = 1;
                        }
                        else if (!found)
                        {
                            tmpneur->confrom[conn].limb = limbnum;
                            found = 1;
                        }
                    }
                    else
                    {
                        if ((tmpneur->confrom[conn].limb == 0) &&
                            (repres[limbnum].origgene !=
                             repres[repres[limbnum].dadnum].origgene))
                        {
                            found = 1;
                        }
                        if (limbnum == 0)
                        {
                            for (i=0; i < MAXLIMBS; i++)
                            {
                                if (repres[i].dead) break;
                                if (repres[i].origgene != repres[repres[i].dadnum].origgene)
                                    if (tmpneur->confrom[conn].limb == 
                                        repres[i].origgene)
                                    {
                                        tmpneur->confrom[conn].limb = i;
                                        found = 1;
                                    }
                            }
                        }
                        if (repres[limbnum].dadnum != -1)
                        {
                            if (tmpneur->confrom[conn].limb == 
                                repres[repres[limbnum].dadnum].origgene)
                            {
                                tmpneur->confrom[conn].limb = 
                                    repres[limbnum].dadnum;
                                found = 1;
                            }
                        }
                        for (i=0; i < MAXLIMBS; i++)
                        {
                            if (repres[i].dead) break;
                            if (repres[i].dadnum == limbnum)
                                if (tmpneur->confrom[conn].limb == 
                                    repres[i].origgene)
                                {
                                    tmpneur->confrom[conn].limb = i;
                                    found = 1;
                                }
                        }
                    }
                    if (!found)
                    {
                        tmpneur->confrom[conn].exists = 0;
                        /*myprintf("Warning: Could not transcribe conn %d:%d in limb %d\n", 
                          neur, conn, limbnum);*/
                    }
                    if (tmpneur->confrom[conn].limb == -1) 
                    {
                        mydie("Conn from -1 !\n");
                    }
		    
                }
            }
        }

    }

    void dev1RecInGenome()
    {
        // finds a gene that carries a recurrence flag (recursive duplication,
        // i.e. segmentation) and turns it into several independent (though
        // identical) genes, which will now be able to evolve independently.
        int i, j, recs[MAXGENES], nbrecs;
        nbrecs=0;
        for (i=1; i < MAXGENES; i++) // YOU CANNOT RECUR LIMB 0
            if ((!genome[i].dead) && (genome[i].recur > 0))
            {
                recs[nbrecs] = i;
                nbrecs++;
            }
        if (nbrecs)
        {
            i = random() % nbrecs;
            Animat tmpA;
            myprintf("\nrec found at gene %d !\n", recs[i]);
            tmpA.copyFrom(this);
            for (j=0; j < MAXLIMBS; j++)
            {
                tmpA.repres[j].dead=1;
                tmpA.repres[j].symmetrised = 0;
            }
            tmpA.transGeneDev1Rec(0, -1, 0, 0, recs[i]);
            tmpA.transNeurons();
            myprintf("\nDeveloped genome has %d limbs\n", tmpA.nblimbs());
            myprintf("\nBefore development:\n", tmpA.nblimbs());
            displayGenomeFull();
            if (tmpA.nblimbs() < MAXGENES)
                for (j=0; j < MAXGENES; j++)
                    genome[j] = tmpA.repres[j];
            else myprintf("No copying, too big\n");
            myprintf("\nAfter development:\n", tmpA.nblimbs());
            displayGenomeFull();
        }
        else myprintf("\nNo rec found to develop !\n");
    }


    void transGeneDev1Rec(int zegene, int zedad, int numrec, int symm, int genetodev)
    {
        // Used in the previous function.

        int limbnum, i;
        //myprintf("transGeneSeg(%d, %d, %d, %d)\n",
        //	zegene, zedad, numrec, symm);
        limbnum = nblimbs();
        if (limbnum >= MAXLIMBS) return;
        repres[limbnum] = genome[zegene];
        repres[limbnum].dead = 0;
        repres[limbnum].origgene = zegene;
        repres[limbnum].dadnum = zedad;
        if (symm)
            repres[limbnum].symmetrised = 1-repres[zedad].symmetrised;
        else
            repres[limbnum].symmetrised = repres[zedad].symmetrised;
        if (repres[limbnum].symmetrised) 
            repres[limbnum].alpha *= -1;
        if (repres[zedad].origgene == zegene) // recursion !
        {
            repres[limbnum].recur = 0;
            repres[zedad].recur= 0; 
            //repres[limbnum].alpha = repres[limbnum].alpharec / 4.0;
            //repres[limbnum].beta = repres[limbnum].betarec / 4.0;
            repres[limbnum].alpha = 0;
            repres[limbnum].beta = 0;
            if (repres[limbnum].symmetrised) 
                repres[limbnum].alpha *= -1;
        }
        for (i=0; i < MAXGENES; i++)
            if ((genome[i].dadnum == zegene) && !genome[i].terminal 
                && !genome[i].dead)
            {
                transGeneDev1Rec(i, limbnum,  genome[i].recur, 0, genetodev);
                //if (genome[i].ref1)
                //    transGeneDev1Rec(i, limbnum,  genome[i].recur, 1, genetodev);
            }
        if (numrec && (zegene == genetodev)) // LIMB 0 CANNOT BE RECURRED (control centre)
        {
            //myprintf("Recur in gene %d\n", zegene);
            transGeneDev1Rec(zegene, limbnum, numrec -1, 0, genetodev);
            limbnum = nblimbs()-1;
        }
        for (i=0; i < MAXGENES; i++)
            if ((genome[i].dadnum == zegene) && genome[i].terminal
                && !genome[i].dead && !numrec)
            {
                transGeneDev1Rec(i, limbnum,  genome[i].recur, 0, genetodev);
                //if (genome[i].ref1)
                //    transGeneDev1Rec(i, limbnum,  genome[i].recur, 1, genetodev);
            }

    }
    
    /*void transGenome(int g, int dad,int malpha, int mbeta, int ignoresym) 
      {
      int i, j, num;
      static int r;
      // initialisation occurs if specified dad is -1
      if (dad == -1)
      {
      for (i=0; i < MAXLIMBS; i++)
      repres[i].dead = 1;
      r = 0;
      }
      //printf ("limb %d, ", r);
      if (r >= MAXLIMBS) return;
      repres[r] = genome[g];
	
      if (malpha == -1) repres[r].symmetrised = 1;
      else repres[r].symmetrised = 0;
	
      repres[r].dadnum = dad;
      repres[r].alpha *= malpha;
      repres[r].beta *= mbeta;
	
      for (i=0; i < MAXNEUR; i++)
      for (j=0; j < MAXCONFROM; j++)
      if (repres[r].neurons[i].confrom[j].limb == genome[g].dadnum)
      repres[r].neurons[i].confrom[j].limb = dad;
      num = r;
      r++;
      if (r >= MAXLIMBS) return;
      for (i=0; i < MAXGENES; i++)
      if ((genome[i].dadnum == g) && (genome[i].dead == 0))
      transGenome(i, num, malpha, mbeta, 0);
      if (!ignoresym)
      {
      if (genome[g].ref1)
      {
      transGenome(g, dad, -malpha, mbeta, 1);
      }
      }
      }*/

    void generate(dReal nx, dReal ny, dReal alpha);
    void remove();
    
    void setPos(dReal nx, dReal ny)
    {
        // set position in the x,y plane
        dReal *trunkpos;
        dReal aabb[6];
        dGeomGetAABB ((dGeomID)space, aabb);
        trunkpos = (dReal *)dBodyGetPosition(limbs[0].id);
        setPos (nx, ny, (trunkpos[2]-aabb[4]) + 0.01);
    }

    void setPos(dReal nx, dReal ny, dReal nz)
    {
        // set position in space
        int i;
        dReal ox, oy, oz;
        dReal *tmpvect, *trunkpos;;
        trunkpos = (dReal *)dBodyGetPosition(limbs[0].id);
        ox = trunkpos[0];
        oy = trunkpos[1];
        oz = trunkpos[2];
        for (i=0; i < MAXLIMBS; i++)
            if (limbs[i].id)
            {
                tmpvect = (dReal *)dBodyGetPosition(limbs[i].id);
                dBodySetPosition (limbs[i].id, nx + (tmpvect[0]-ox), 
                                  ny + (tmpvect[1]-oy), nz + (tmpvect[2]-oz));
            }
    }

    void rotateByQ(dQuaternion q)
    {
        // rotate me by Quaternion q's angle
        // 
        // get Pos of each limb, in limb 0's frame of ref
        // rotate limb 0
        // for each limb:
        // convert saved relative Pos to global coords, apply
        // rotate body of limb
        dVector3 oldpos[MAXLIMBS];
        dQuaternion qtmp;
        dVector3 postmp;
        for (int i=0; i < MAXLIMBS; i++)
            if (limbs[i].id)
                dBodyGetPosRelPoint(limbs[0].id,
                                    dBodyGetPosition(limbs[i].id)[0],
                                    dBodyGetPosition(limbs[i].id)[1],
                                    dBodyGetPosition(limbs[i].id)[2],
                                    oldpos[i]);
        for (int i=0; i < MAXLIMBS; i++)
            if (limbs[i].id)
            {
                // qtmp = q * body's current quaternion
                dQMultiply0(qtmp, q, dBodyGetQuaternion(limbs[i].id));
                dBodySetQuaternion(limbs[i].id, qtmp);
            }
        for (int i=1; i < MAXLIMBS; i++)
            if (limbs[i].id)
            {
                dBodyGetRelPointPos(limbs[0].id,
                                    oldpos[i][0],
                                    oldpos[i][1],
                                    oldpos[i][2],
                                    postmp);
                dBodySetPosition(limbs[i].id, 
                                 postmp[0], postmp[1], postmp[2]);
            }
    }

    void dropRandomOnSphere()
    {
        // drop me at random over the sphere
        //
        //Of course, I must be alive (i.e. generated), and the world must be
        //spherical  !
        // 
        // How it works:
        // 1 - pick a point at random in the 3D sphere centred at (0,0,0)
        // 2 - calculate the latitude and longitude of that point
        // 3 - drop on the sphere at that lati/longi
        // Why we need this: dropping at random lati/longi would result in 
        // "crowded poles" (at smaller latitudes, a certain longitude diff
        // corresponds to a smaller distance)
        dReal x, y, z, dist, lati, longi;
        if (WORLDTYPE != SPHERICWORLD)
        {
            myprintf("Trying to drop me on a flat world !\n");
            mydie("Damn! Trying to drop animat on a flat world !\n");
        }
        do{
            do{
                x = (dReal) (10000 - random() % 20000) / 100.0;
                y = (dReal) (10000 - random() % 20000) / 100.0;
                z = (dReal) (10000 - random() % 20000) / 100.0;
            }
            while (sqrt(x*x + y*y + z*z) > 9999);

            dist = sqrt(x*x + y*y + z*z);
            lati = acos(z / dist); // z / R
            // distance from the z axis
            dist = sqrt(x*x + y*y);
            longi = asin(x / dist);
            if (y > 0) longi = M_PI - longi;
            dropLatiLongi(lati, longi);
            AIRCOLLISIONS = 0;
            for (int i=0; i < REGISTSIZE; i++)
                if ((regist[i]) && (regist[i] != this))
                {
                    dSpaceCollide2((dxGeom*)(space), 
                                   (dxGeom*)(regist[i]->space), 0,
                                   &airCallback);
                }
            myprintf("Trying\n");
        } while (AIRCOLLISIONS);
        AIRCOLLISIONS = 0;
        dJointGroupEmpty (contactgroup);
    }
    
    dReal getLongi()
    {
        // get my current longitude (in a spherical world)
        dReal *pos = (dReal *) dBodyGetPosition(limbs[0].id);
        dReal x = pos[0]; dReal y = pos[1]; 
        dReal dist = sqrt(x*x + y*y);
        dReal longi = asin (x / dist);
        if (y > 0)  longi = M_PI - longi;
        return longi;
    }
    dReal getLati()
    {
        // get my current latitude (in a spherical world)
        dReal *pos = (dReal *) dBodyGetPosition(limbs[0].id);
        dReal x = pos[0]; dReal y = pos[1]; dReal z = pos[2];
        dReal dist = sqrt(x*x + y*y + z*z);
        return acos (z / dist);
    }
    
    void dropLatiLongi(dReal lati, dReal longi)
    {
        // drop me at a certain latitude an longitude, on a spherical world
        //
        // NOTE: north pole = latitude 0, south pole = latitude M_PI
        // Longi: "Greenwich" (longitude 0) is the negative-y portion of the
        // intersec of the sphere with
        // the Oyz plane (i.e. it's the vertical half-meridian that lies
        // directly on the left of the sphere when seeing it from a point on
        // the positive half of the x axis).	
        dVector3 v;
        setPos(0, 0);
        // First we're putting the creature in the right orientation
        dReal zoff = dBodyGetPosition(limbs[0].id)[2];
        dQuaternion q;
        dQFromAxisAndAngle(q, 1, 0, 0, lati);
        rotateByQ(q);
        dQFromAxisAndAngle(q, 0, 0, 1, longi);
        rotateByQ(q);
        // Now the position vector we want is simply the (0,0, RADIUS + zoff)
        // vector from limb 0, translated into global coords (because the
        // creature has the correct orientation).
        dBodyVectorToWorld(limbs[0].id, 0, 0, WORLDRADIUS + zoff + DROPHEIGHT, v);
        setPos(v[0], v[1], v[2]);
    }
    
    int test_for_intra_coll()
    {
        // test whether my limbs are colliding with each other
	
        int a, b, res;
        res = 0;
        dSpaceCollide (globalspace,0,&nearCallback);
        for (a=0; a < nblimbs(); a++)
        {
            for (b=0; b < nblimbs(); b++)
            {
                // If two bodies are connected by anything else than a 
                // Hinge, it is a non-allowed contact, because the 
                // collision callback does not add contact joints between
                // hinge-connected limbs.
                if (dAreConnectedExcluding(limbs[a].id, limbs[b].id,
                                           dJointTypeHinge))
                {
                    //myprintf("IntraColl: %d, %d !!\n",a, b);
                    /*int i, nbj; dJointID jointid;
                      nbj = dBodyGetNumJoints(limbs[a].id);
                      for (i=0; i < nbj; i++)
                      {
                      jointid = dBodyGetJoint(limbs[a].id, i);
                      if ((dJointGetBody(jointid, 0) == limbs[b].id)
                      || (dJointGetBody(jointid, 1) == limbs[b].id))
                      {
                      printf("Joint between %d and %d of type %d (%d)\n",
                      a, b, dJointGetType(jointid), 
                      dJointTypeContact);
                      }
                      }*/
                    res = 1;
                }
            }
        }
        dJointGroupEmpty (contactgroup);
        return res;
    }

}; 


int board [BSIDE][BSIDE];

void addDamage(Animat *A, Animat *B, dReal damage)
{
    // utility function: takes note that A deals damage to B
    for (int i=0; i < REGISTSIZE; i++)
    {
        if (!regist[i]) continue;
        if (regist[i] == A)
            for (int j=0; j < REGISTSIZE; j++)
            {
                if (!regist[j]) continue;
                if (regist[j] == B)
                    DAMAGETABLE[i][j] += damage;
            }
    }
}


void Animat::fillSensors()
{
    // update sensors information
    
    // note: if you're going to add / change available sensors, be sure to also
    // change SENSORTYPES and NBSENSORTYPES at the beginning of this file.

    int i, neur;
    dReal xball=0, yball=0,  xother=0, yother=0;
    dVector3 vectball, vectother;
    dReal *posball=NULL, *posother=NULL;
    i=0;

    int others=0;;
    
    // Are there other individuals in the simulation - or am I alone in the
    // universe? 
    for (int i=0; i < REGISTSIZE; i++)
        if (regist[i] && (regist[i] != this))
        {
            others = 1;
            break; 
        }
    
    for (i=0; i < nblimbs(); i++)
    {

        if (i != 0)
            repres[i].neurons[2].out = 
                dJointGetHingeAngle(limbs[i].joint) 
                / M_PI;


        // We don't need to calculate the sensor inputs if there are no
        // sensors for this type of inputs in this limb...
        int needball=0;
        int needotheranim=0;
        for (neur=0; neur < MAXNEUR; neur++)
        {
            if ((repres[i].neurons[neur].type == SENSBALLX)
                || (repres[i].neurons[neur].type == SENSBALLY))
                needball = 1;
            if ((repres[i].neurons[neur].type == SENSCLOSESTANIMX)
                || (repres[i].neurons[neur].type == SENSCLOSESTANIMY))
                needotheranim = 1;
        }

        /*
        // Old, "complex" external sensors :
        // Abs. value: inv distance to the target.
        // Sign: whether the organism is "left" or "right" of the limb.
        // Basically, the sign of the value is such that, if the actuator
        // receives only a value of the same sign, the limb will go "towards"
        // the target (regardless of orientation, reflection, etc.)
        // These are included mostly for historical reasons. They seemed to
        // bring no significant improvement over simple Sims-like x- and
        // y-sensors.
        //
        if (BALL && needball)
        {
	    posball = (dReal*) dBodyGetPosition(ball.body);
	    dBodyGetPosRelPoint(limbs[i].id, posball[0], 
        posball[1], posball[2], vectball);

	    xball = vectball[0]; yball = vectball[1]; 
	    zball = vectball[2];

	    distball = 
		1.0 / sqrt (xball*xball + yball * yball + zball * zball);
	    if (repres[i].orient == 1)
	    {
		if (repres[i].symmetrised)
		{
        if (yball > 0) distball = -distball;
		}
		else
		{
        if (yball < 0) distball = -distball;
		}
	    }
	    else
	    {
		if (zball < 0) distball = -distball;
	    }
	    distball = tanh(distball/2.0);
        }
        */
	
	
        // External sensors: simple X or Y distance to the trunk root of the
        // closest animat, within the frame of reference of this limb, squashed
        // through tanh()
        // 
        if (BALL && needball)
        {
            posball = (dReal*) dBodyGetPosition(ball.body);
            dBodyGetPosRelPoint(limbs[i].id, posball[0], 
                                posball[1], posball[2], vectball);

            xball = vectball[0]; yball = vectball[1]; 

            //xball = tanh(vectball[0] / 2.0);
            //yball = tanh(vectball[1] / 2.0);
            xball = tanh(vectball[0] / 5.0) / 2.0;
            yball = tanh(vectball[1] / 5.0) / 2.0;
            //if (genome[0].symYinputs)
            if (repres[i].symmetrised)
            {
                yball = 0 - yball; // ...?
                //myprintf("Limb %d has inverted Y-ball inputs\n", i);
            }

        }
        if (others && needotheranim)
        {
            posother= (dReal*) dBodyGetPosition(
                closestanimat->limbs[0].id);
            //posother= (dReal*) dBodyGetPosition(other->limbs[0].id);
            dBodyGetPosRelPoint(limbs[i].id, posother[0], 
                                posother[1], posother[2], vectother);

            xother = tanh(vectother[0] / 5.0) / 2.0;
            yother = tanh(vectother[1] / 5.0) / 2.0;
            //if (genome[0].symYinputs)
            if (repres[i].symmetrised)
            {
                yother = 0 - yother; // ...?
                //myprintf("Limb %d has inverted Y-other inputs\n", i);
            }

        }

        for (neur=0; neur < MAXNEUR; neur++)
        {
            if (!repres[i].neurons[neur].exists) continue;
            if (repres[i].neurons[neur].type == SENSDAMAGE)
            {
                repres[i].neurons[neur].out = 
                    tanh(log(limbs[i].damage / 5.0 + 1.0));
                if (DISABLESENSORS)
                    repres[i].neurons[neur].out = 0;
            }
            if (repres[i].neurons[neur].type == SENSTOUCH)
            {
                if (limbs[i].touchesOther)
                    repres[i].neurons[neur].out = 1;
                else
                    repres[i].neurons[neur].out = 0;
                if (DISABLESENSORS)
                    repres[i].neurons[neur].out = 0;
            }
            if (repres[i].neurons[neur].type == SENSCLOSESTANIMX)
            {
                repres[i].neurons[neur].out = xother;
                if (DISABLESENSORS)
                    repres[i].neurons[neur].out = 0;
            }
            if (repres[i].neurons[neur].type == SENSCLOSESTANIMY)
            {
                repres[i].neurons[neur].out = yother;
                if (DISABLESENSORS)
                    repres[i].neurons[neur].out = 0;
            }
            if (repres[i].neurons[neur].type == SENSBALLY)
            {
                if (BALL) repres[i].neurons[neur].out = yball;
                else repres[i].neurons[neur].out = 0;
                if (DISABLESENSORS)
                    repres[i].neurons[neur].out = 0;
                //myprintf("%.6f\n", repres[i].neurons[neur].out);
            }
            if (repres[i].neurons[neur].type == SENSBALLX)
            {
                if (BALL) repres[i].neurons[neur].out = xball;
                else repres[i].neurons[neur].out = 0;
                if (DISABLESENSORS)
                    repres[i].neurons[neur].out = 0;
                //myprintf("%.6f\n", repres[i].neurons[neur].out);
            }
            if (repres[i].neurons[neur].type == SENSUPDOWN)
            {
                repres[i].neurons[neur].out = upDown;
                if (DISABLESENSORS)
                    repres[i].neurons[neur].out = 0;
                //myprintf("%.6f\n", repres[i].neurons[neur].out);
            }
            if (repres[i].neurons[neur].type == SENSLEFTRIGHT)
            {
                repres[i].neurons[neur].out = leftRight;
                if (DISABLESENSORS)
                    repres[i].neurons[neur].out = 0;
                //myprintf("%.6f\n", repres[i].neurons[neur].out);
            }
        }
    }
}



void Animat::remove()
{
    // remove me from the simulation
    
    int i;
    //myprintf("Remove\n");
    if (!alive) 
    {
        mydie("Damn ! Trying to remove a non-living animat !\n"); 
    }
    
    dSpaceDestroy(space);
    resetDamages();
    for (i=0; i < MAXLIMBS; i++)
    {
        if (limbs[i].id)
        {
            if (limbs[i].joint) 
            {
                //printf ("%d\n ", (int)limbs[i].joint);
                dJointDestroy (limbs[i].joint);
                limbs[i].joint = 0;
            }
            dBodyDestroy(limbs[i].id);
            limbs[i].id = 0;
        }
    }
    i=0; while ((i < REGISTSIZE) && (regist[i] != this)) i++;
    if (i >= REGISTSIZE) 
    { 
        mydie("Damn ! Couldn't find animat in register while removing !\n");
    }
    regist[i] = NULL;
    alive = 0;
}

void Animat::generate(dReal nx, dReal ny, dReal alpha)
{

    // put me into the simulation (make me "alive")
    
    // Assumes that the repres has been filled beforehand !
    int i, j;
    dReal aabb[6];
    dReal *trunkpos;
    //myprintf("Generating..\n");
    if (alive) 
    { myprintf("Damn ! Re-Generating a living animat !"); return; }
    space = dSimpleSpaceCreate(globalspace);
    idxgen = 0;
    transGenome();
    printGenome(); printRepres();
    //displayRepres();
    readRepres (0, 0, 0, alpha);  
    dGeomGetAABB ((dGeomID)space, aabb);
    trunkpos = (dReal *)dBodyGetPosition(limbs[0].id);
    setPos (nx, ny, (trunkpos[2]-aabb[4]) + 0.01);
    for (i=0; i < MAXLIMBS; i++)
        for (j = 0; j < MAXNEUR; j++)
        {
            repres[i].neurons[j].state = 0; 
            repres[i].neurons[j].out=0; 
        }
    for (i=0; i < MAXLIMBS; i++)
        limbs[i].oldspeed = 0; // doing this in the constructor has no effect!?
    i = 0; while ((regist[i]) && (i < REGISTSIZE)) i++;
    if (i >= REGISTSIZE) 
        mydie("Damn ! Register full !\n"); 
    regist[i] = this;
    alive = 1;
}

    


    
void anglesToPoint(dBodyID id, dReal l, dReal w, dReal h, 
                   dReal alpha, dReal beta, dVector3 result)
{

    // What is the intersection of a certain line, from the centre of the limb,
    // with horizontal angle alpha and vertical angle beta, and the surface of
    // the limb? 
    
    dReal d;
    dReal x, y, z;
    
    l = l / 2.0; w = w / 2.0; h = h / 2.0; 
    d = sqrt (h*h + w*w + l*l);
    z = d * sin(beta);
    d = d * cos(beta);  // => d = sqrt(x*x + y*y) 
    x = d * cos(alpha);
    y = d * sin(alpha);
    if (x > l) x = l;
    if (y > w) y = w;
    if (z > h) z = h;
    if (x < -l) x = -l;
    if (y < -w) y = -w;
    if (z < -h) z = -h;
    dBodyGetRelPointPos (id, x, y, z, result);
}


limb *Animat::createLimb (int index, dReal  zel, dReal zew, dReal zeh, 
                          dReal x, dReal y, dReal z, dReal alpha)
{
    // create a new limb, using parameters for morphological data
    
    limb *res;
    dReal l, w, h;
    dMatrix3 R1;
    l = 0.2 + zel;
    w = 0.2 + zew;
    h = 0.2 + zeh;
    if (index >= MAXLIMBS) 
        mydie("Damn ! Too many limbs !\n"); 
    res = &limbs[index];
    if (res->id) 
        mydie("Damn ! Limb %d exists\n",index);
    res->lgt = l; res->wdt = w; res->hgt = h;
    res->id = dBodyCreate(world);
    res->index = index;
    res->immunitytimer=0;
//    dMassSetBox (&res->mass,1, l / 2.0, w / 2.0, h / 2.0);
    dMassSetBox (&res->mass,1, l, w, h);
//    dMassAdjust(&res->mass, (l + w + h) / 10.0);
    dMassAdjust(&res->mass, (l+w+h)/3.0);
    res->alpha = 0; res->beta = 0;
    res->sides[0] = l; res->sides[1] = w; res->sides[2] = h;
    res->damage = 0;
    res->damagedone = 0;
    res->owner = this;
    
    res->geom = dCreateBox (space, l, w, h);

    // Each geom has a pointer to the corresponding limb, which is useful for
    // damage calculation
    dGeomSetData (res->geom, res);
    //myprintf("%d %d\n", index, res);

    dGeomSetBody (res->geom, res->id); 
   
    dBodySetMass(res->id, &res->mass);
    dBodySetPosition   (res->id, x, y, z);
    dRFromAxisAndAngle (R1, 0, 0, 1, alpha);
    dBodySetRotation (res->id, R1);
    //dBodySetPosition (res->id, x, y, z);
    
    res->dad = 0;
    return res;
}

limb *Animat::addLimb (limb *trunk, int idxGen)
{
    // add a new limb, reading the data from the repres at position idxGen
    
    dReal alpha, beta; int orient;
    dReal size = 0;
    dReal *tmpmat;
    dQuaternion q1, q2, q3;
    dVector3 tmpvect;
    limb *leg;
    
    /* discretization
       myprintf("\nOld alpha = %.3f\n", repres[idxGen].alpha);
       alpha = 4 * repres[idxGen].alpha;
       alpha = rint(alpha);
       alpha = alpha / 4.0;
       beta = 4 * repres[idxGen].beta;
       beta = rint(beta);
       beta = beta / 4.0;*/

    alpha =  2.0 * M_PI * repres[idxGen].alpha;
    beta =  2.0 * M_PI * repres[idxGen].beta;

    orient = repres[idxGen].orient;
    leg = createLimb(idxGen, repres[idxGen].lgt, repres[idxGen].wdt, 
                     repres[idxGen].hgt, 0, 0, 0, 0);
    leg->alpha = alpha; leg->beta = beta;
//myprintf("%f \n", alpha);
    anglesToPoint(trunk->id, trunk->lgt, trunk->wdt, trunk->hgt, 
                  alpha, beta, tmpvect);
    
    tmpmat = (dReal *) dBodyGetRotation (trunk->id);
    dQFromAxisAndAngle(q3, tmpmat[2], tmpmat[6], tmpmat[10], alpha);  
    dRtoQ (dBodyGetRotation (trunk->id), q2);
    dQMultiply0 (q1, q3, q2);  // q1 = q3 * q2
    dBodySetQuaternion (leg->id, q1);
    tmpmat = (dReal *) dBodyGetRotation (leg->id);
    //dQFromAxisAndAngle (q3, -tmpmat[4], -tmpmat[5], -tmpmat[6], beta);
    dQFromAxisAndAngle (q3, -tmpmat[1], -tmpmat[5], -tmpmat[9], beta);
    dRtoQ (dBodyGetRotation (leg->id), q2);
    dQMultiply0 (q1, q3, q2);  // q1 = q2 * q3
    dBodySetQuaternion (leg->id, q1);
    
    
    dBodySetPosition (leg->id, tmpvect[0], tmpvect[1], tmpvect[2]);
    // We must push the leg half it length in its own x direction
    // So that it "only just" touches the trunk at contact point
    tmpmat = (dReal *) dBodyGetRotation (leg->id);
    size = (repres[idxGen].lgt + 0.2) / 2;
    tmpvect[0] += tmpmat[0]*size;    
    tmpvect[1] += tmpmat[4]*size;    
    tmpvect[2] += tmpmat[8]*size;    
    dBodySetPosition (leg->id, tmpvect[0], tmpvect[1], tmpvect[2]);
    
    /* OLD !
    // Obtaining the coords of the vector between trunk center and anchor point
    *trunkpos = (dReal *) dBodyGetPosition (trunk->id);
    for (i=0; i < 3; i++) tmpvect[i] -= trunkpos[i];
    for (i=0; i < 3; i++) size += tmpvect[i] * tmpvect[i];
    size = sqrt(size);
    size = size / (leg->lgt / 2.0);
    for (i=0; i < 3; i++) tmpvect[i] = tmpvect[i] / size;
    limbpos = (dReal *) dBodyGetPosition (leg->id);
    for (i=0; i < 3; i++) tmpvect[i] = limbpos[i] + tmpvect[i];
    dBodySetPosition (leg->id, tmpvect[0], tmpvect[1], tmpvect[2]);*/
    
    
    anglesToPoint(trunk->id, trunk->lgt, trunk->wdt, trunk->hgt, alpha, beta, tmpvect);
    
    tmpmat = (dReal *) dBodyGetRotation (leg->id);
    leg->joint = dJointCreateHinge (world, 0);
    dJointAttach (leg->joint, trunk->id, leg->id);
    dJointSetHingeAnchor (leg->joint, tmpvect[0], tmpvect[1], tmpvect[2]);
    if (orient == 1)
    {
        if (repres[idxGen].symmetrised)
            dJointSetHingeAxis (leg->joint, tmpmat[2], tmpmat[6], tmpmat[10]);
        else
            dJointSetHingeAxis (leg->joint, -tmpmat[2], -tmpmat[6], -tmpmat[10]);
    }
    else
        dJointSetHingeAxis (leg->joint, tmpmat[1], tmpmat[5], tmpmat[9]);
    dJointSetHingeParam(leg->joint, dParamVel, 0);
    dJointSetHingeParam(leg->joint, dParamFMax, MAXFORCE);
 
    dJointSetHingeParam(leg->joint, dParamFudgeFactor, 0.3);
    dJointSetHingeParam(leg->joint, dParamLoStop, -0.75* M_PI);
    dJointSetHingeParam(leg->joint, dParamHiStop, 0.75* M_PI);

    leg->dad = trunk;
    
    return leg;
}

void Animat::readRepres(dReal x, dReal y, dReal z, dReal alpha)
{

    // create me within the simulation, by reading morphological data from the
    // repres (used by generate(), shouldn't be used directly)
    
    int idx;
    limb *cur;
//    myprintf("Creating Dad ...\n");
    cur = createLimb(0, repres[0].lgt, 
                     repres[0].wdt, 
                     repres[0].hgt,
                     x, y, z, alpha);
    for (idx=1; idx < MAXLIMBS; idx++)
    {
        if (repres[idx].dead) break;
        if (idx >= MAXLIMBS) break;
        cur = addLimb(&limbs[repres[idx].dadnum], idx);
//	myprintf("Adding Son (idx=%d)...\n", idx);
    }
    myprintf("Created, break! (idx=%d)...\n", idx);
}


// old utility functions, create features in the (flat!) environment

#ifdef DRAWSTUFF
void drawCorridor()
{
    if (!CORRIDOR) return;
    static const dReal corr[3] = {100,1,50};
    dsSetColor(1,0,1);
    dsDrawBox (dGeomGetPosition(rcorr), dGeomGetRotation(rcorr), corr);
    dsDrawBox (dGeomGetPosition(lcorr), dGeomGetRotation(lcorr), corr);
}
#endif
void makeCorridor()
{
    if (!CORRIDOR) return;
    lcorr = dCreateBox (globalspace, 100, 1, 50);
    rcorr = dCreateBox (globalspace, 100, 1, 50);
    dGeomSetPosition (rcorr, 0, ((float)CORRSPACE / 2.0) + 0.6, 1); 
    dGeomSetPosition (lcorr, 0, -((float)CORRSPACE / 2.0) - 0.6, 1); 
}
void makeWalls()
{
    if (WALLS==0) return;
    walln = dCreateBox (globalspace, 1, WALLSIZE, 30);
    walls = dCreateBox (globalspace, 1, WALLSIZE, 30);
    wallw = dCreateBox (globalspace, WALLSIZE, 1, 30);
    walle = dCreateBox (globalspace, WALLSIZE, 1, 30);
    dGeomSetPosition (walln, -(WALLSIZE/2.0), 0, 15); 
    dGeomSetPosition (walls, (WALLSIZE/2.0), 0, 15); 
    dGeomSetPosition (wallw, 0, (WALLSIZE/2.0), 15); 
    dGeomSetPosition (walle, 0, -(WALLSIZE/2.0), 15); 
}
#ifdef DRAWSTUFF
void drawWalls()
{
    if (!WALLS) return;
    const dReal ns[3] = {1,WALLSIZE,30};
    const dReal ew[3] = {WALLSIZE,1,30};
    dsSetColor(1,0,1);
    dsDrawBox (dGeomGetPosition(walln), dGeomGetRotation(walln), ns);
    dsDrawBox (dGeomGetPosition(walls), dGeomGetRotation(walls), ns);
    dsDrawBox (dGeomGetPosition(walle), dGeomGetRotation(walln), ew);
    dsDrawBox (dGeomGetPosition(wallw), dGeomGetRotation(walls), ew);
}
#endif
void resetBoard() 
{ 
    int i, j; 
    for (i=0; i < BSIDE; i++) for (j=0; j < BSIDE; j++) board[i][j] = 1;
}
void makeBoard()
{
    if (BOARD==0) return;
    walln = dCreateBox (globalspace, 1, BSIDE*SQSIZE, 30);
    walls = dCreateBox (globalspace, 1, BSIDE*SQSIZE, 30);
    wallw = dCreateBox (globalspace, BSIDE*SQSIZE, 1, 30);
    walle = dCreateBox (globalspace, BSIDE*SQSIZE, 1, 30);
    dGeomSetPosition (walln, 0, BSIDE*SQSIZE / 2, 15); 
    dGeomSetPosition (walls, BSIDE*SQSIZE, BSIDE*SQSIZE/2, 15); 
    dGeomSetPosition (wallw, BSIDE*SQSIZE/2, 0, 15); 
    dGeomSetPosition (walle, BSIDE*SQSIZE/2, BSIDE*SQSIZE, 15); 
    resetBoard();
}

#ifdef DRAWSTUFF
void drawBoard()
{
    if (!BOARD) return;
    int i, j;
    const dReal ns[3] = {1,BSIDE*SQSIZE,30};
    const dReal ew[3] = {BSIDE*SQSIZE,1,30};
    const dReal sqsides[3] = {SQSIZE-0.2,SQSIZE-0.2,0.05};
    dMatrix3 IdMat = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0};
    dReal tab1[3] = {0, 0, 0.05};
    dsSetColor(1,0,1);
    dsDrawBox (dGeomGetPosition(walln), dGeomGetRotation(walln), ns);
    dsDrawBox (dGeomGetPosition(walls), dGeomGetRotation(walls), ns);
    dsDrawBox (dGeomGetPosition(walle), dGeomGetRotation(walln), ew);
    dsDrawBox (dGeomGetPosition(wallw), dGeomGetRotation(walls), ew);
    for (i=0; i < BSIDE; i++)
        for (j=0; j < BSIDE; j++)
        {
            if (board[i][j]) dsSetColor (1, 1, 0);
            else dsSetColor (0, 1, 1);
            tab1[0] = i * SQSIZE + SQSIZE / 2;
            tab1[1] = j * SQSIZE + SQSIZE / 2;
            dsDrawBox (tab1, IdMat, sqsides);
        }
}
#endif


#ifdef DRAWSTUFF
// start simulation - set viewpoint
static void start()
{
    float xyz[3] = {6,0,30};
    float hpr[3] = {-179.9,-57,0};
    if (WORLDTYPE == FLATWORLD)
    {
        xyz[0] = 1.7; xyz[1] = -7; xyz[2] = 6.8;
        hpr[0] = 104; hpr[1] = -44; hpr[2] = 0.0;
    }
    dsSetViewpoint (xyz,hpr);
    printf ("Simulation started !");
}


// called when a key pressed

static void command (int cmd)
{
    if (cmd == 'e' || cmd == 'E') {
        myprintf("You pressed 'E'\n");
    }
}
#endif


static void airCallback (void *data, dGeomID o1, dGeomID o2)
{
    // utility function, 
    // used for detection of collisions in mid-air when dropped
    if (dGeomIsSpace (o1) || dGeomIsSpace (o2)) 
    {
        mydie("Damn, one of two colliders is a space !\n");
    }
    if (dCollide(o1,o2,MAXCONT,&contact[0].geom,sizeof(dContact))) 
    {
        AIRCOLLISIONS = 1;
        return;
    }
}

// Callback for collisions
static void nearCallback (void *data, dGeomID o1, dGeomID o2)
{
    //if ((o1 != sphereID) && (o2 != sphereID)) return;

    if (dGeomIsSpace (o1) || dGeomIsSpace (o2)) 
    {
        //if ((o1 == sphereID) || (o2 == sphereID)) myprintf("Ground\n");
        dSpaceCollide2 (o1,o2,data,&nearCallback);
        if (dGeomIsSpace (o1)) dSpaceCollide ((dxSpace*)o1,data,&nearCallback);
        if (dGeomIsSpace (o2)) dSpaceCollide ((dxSpace*)o2,data,&nearCallback);
    }
    else
    {
        //myprintf("Collide\n");
        dBodyID b1 = dGeomGetBody(o1);
        dBodyID b2 = dGeomGetBody(o2);

      
        // if the two bodies are already connected by
        //  any type of joint, whether hinge or contact, we skip this pair -
        //  barbarian style.
        //  It makes damage calculation much simpler.
      

        // the first conditional test is necessary because if either o1 or o2 is
        // a non-body geom (i.e. the ground or sphere geom), the debug build of
        // ODE crashes out with message: ODE INTERNAL ERROR 2: Bad argument(s)
        // in dAreConnected()
        if (b1 && b2) 
            if (dAreConnected(b1, b2)) 
                return;

        if (int numc = dCollide(o1,o2,MAXCONT,&contact[0].geom,sizeof(dContact))) 
        {
            for (int i=0; i<numc; i++) {
                dJointID c = dJointCreateContact (world,contactgroup,contact+i);
                dJointAttach (c,b1,b2);
            }

            // Damage calculation
	  
            if ((dGeomGetSpace(o1) != dGeomGetSpace(o2))
                && (o1 != sphereID) && (o2 != sphereID)
                && (o1 != groundID) && (o2 != groundID)
                && (o1 != ball.geom) && (o2 != ball.geom)
                )
            {
                //myprintf("Contact!\n");
                if (!dGeomGetData(o1) || !dGeomGetData(o2))
                    mydie("Damn ! No limb data associated to bodies !\n");
                limb *l1 = (limb*) dGeomGetData(o1);
                limb *l2 = (limb*) dGeomGetData(o2);
	      
                l1->touchesOther = 1;
                l2->touchesOther = 1;
	      
                //myprintf("%d\n", l1->index);
                //myprintf("%d\n", l2->index);
	      

                // if none of the limbs are "immunised"....
                if (!(l1->immunitytimer) && !(l2->immunitytimer) &&
                    // and if it is a new contact...
                    (! dBoxTouchesBox(l1->oldpos, l1->oldrot, l1->sides, 
                                      l2->oldpos, l2->oldrot, l2->sides)))
                {
                    //myprintf("New Contact!\n");
                    // finding the current depth of penetration
                    dReal depth0 = 0;
                    for (int i=0; i< numc; i++) 
                    {
                        if (contact[i].geom.depth < 0) 
                            mydie("Negative Depth !\n");
                        if (contact[i].geom.depth > depth0)
                            depth0 = contact[i].geom.depth;
                    }

                    // findung how much penetration there would be if either
                    // went on with its course, while the other stands still;
                    // the largest depth gives the "winner" (the one which goes
                    // "most" towards the other)
		  
                    dGeomID new1, new2;
                    new1 = dCreateBox(alternatespace,  l1->sides[0],
                                      l1->sides[1], l1->sides[2]);
                    new2 = dCreateBox(alternatespace,  l2->sides[0],
                                      l2->sides[1], l2->sides[2]);
                    dGeomSetPosition(new1,
                                     dBodyGetPosition(l1->id)[0] +  STEP * dBodyGetLinearVel(l1->id)[0],
                                     dBodyGetPosition(l1->id)[1] +  STEP * dBodyGetLinearVel(l1->id)[1],
                                     dBodyGetPosition(l1->id)[2] +  STEP * dBodyGetLinearVel(l1->id)[2]);
                    dGeomSetPosition(new2,
                                     dBodyGetPosition(l2->id)[0] +  STEP * dBodyGetLinearVel(l2->id)[0],
                                     dBodyGetPosition(l2->id)[1] +  STEP * dBodyGetLinearVel(l2->id)[1],
                                     dBodyGetPosition(l2->id)[2] +  STEP * dBodyGetLinearVel(l2->id)[2]);
                    dGeomSetRotation(new1, dBodyGetRotation(l1->id));
                    dGeomSetRotation(new2, dBodyGetRotation(l2->id));
                    dReal depth1 = 0; 
                    dReal depth2 = 0;
                    int tmpnumc;
                    tmpnumc = dCollide(new1, o2, MAXCONT,
                                       &contact[0].geom,sizeof(dContact)) ;
                    if (tmpnumc)
                    {
                        for (int i=0; i< tmpnumc; i++) 
                            if (contact[i].geom.depth > depth1)
                                depth1 = contact[i].geom.depth;
                    }
                    tmpnumc = dCollide(o1, new2, MAXCONT,
                                       &contact[0].geom,sizeof(dContact)) ;
                    if (tmpnumc)
                    {
                        for (int i=0; i< tmpnumc; i++) 
                            if (contact[i].geom.depth > depth2)
                                depth2 = contact[i].geom.depth;
                    }

                    // determine the amount of damage
		  
                    // damage apportioned to both

                    if ((depth1 - depth0) > 0) 
                    {
                        l2->immunitytimer = HURTIMMUNITYTIMER;
                        if (VISUAL) myprintf("dam to l2: %f\n", 
                                             (depth1 - depth0));
                        l2->damage += depth1 - depth0;
                        l2->curdamage += depth1 - depth0;
                        l1->damagedone += depth1 - depth0;
                        addDamage(l1->owner, l2->owner, depth1 - depth0);
                    }
                    if ((depth2 - depth0) > 0) 
                    {
                        l1->immunitytimer = HURTIMMUNITYTIMER;
                        if (VISUAL) myprintf("dam to l1: %f\n", 
                                             (depth2 - depth0));
                        l1->damage += (depth2 - depth0);
                        l1->curdamage += (depth2 - depth0);
                        l2->damagedone += (depth2 - depth0);
                        addDamage(l2->owner, l1->owner, (depth2 - depth0));
                    }
		  
                    /*if (depth1 > depth0) 
                      {
                      l2->curdamage = depth1 - depth0;
                      //myprintf("dam to l2: %f\n", l2->curdamage);
                      l2->damage += l2->curdamage;
                      }
                      if (depth2 > depth0) 
                      {
                      l1->curdamage = depth2 - depth0;
                      //myprintf("dam to l1: %f\n", l1->curdamage);
                      l1->damage += l1->curdamage;
                      }*/
#ifdef DRAWSTUFF
                    if (VISUAL)
                    {
                        dsSetColor(.3,.3,.3);
                        dsDrawBox (dGeomGetPosition(new1),
                                   dGeomGetRotation(new1),
                                   l1->sides);
                        dsSetColor(.8,.8,.8);
                        dsDrawBox (dGeomGetPosition(new2),
                                   dGeomGetRotation(new2),
                                   l2->sides);
                    }
#endif

                    dGeomDestroy(new1);
                    dGeomDestroy(new2);
                }
                /*
                  int deepest = 0;
                  dReal oldp1[3], oldp2[3];
                  dReal newp1[3], newp2[3];
                  limb *l1, *l2;
                  for (int i=0; i<numc; i++) {
                  if (contact[i].geom.depth < 0) mydie("Negative Depth !\n");
                  if (contact[i].geom.depth > contact[deepest].geom.depth)
                  deepest = i;
                  }
                  if (!dGeomGetData(o1) || !dGeomGetData(o2))
                  mydie("Damn ! No data associated to bodies !\n");
                  l1 = (limb*) dGeomGetData(o1);
                  l2 = (limb*) dGeomGetData(o2);

                  for (int i=0; i < 3; i++)
                  {
                  // we push the boxes just enough that they don't
                  // intersect any more...
                  oldp1[i] = dBodyGetPosition(l1->id)[i];
                  oldp1[i] +=  contact[deepest].geom.depth *
                  contact[deepest].geom.normal[i] / 1.99;
                  oldp2[i] = dBodyGetPosition(l2->id)[i];
                  oldp2[i] -=  contact[deepest].geom.depth *
                  contact[deepest].geom.normal[i] / 1.99;
                  }
	      
                  for (int i=0; i < 3; i++)
                  {
                  // Then we calculate their position at the next timestep
                  // with their current speed
	      
                  if (VISUAL)
                  {
                  dsSetColor(.4,.4,.4);
                  dsDrawBox (modp1,dBodyGetRotation(l1->id),l1->sides);
                  dsSetColor(.7,.7,.7);
                  dsDrawBox (modp2,dBodyGetRotation(l2->id),l2->sides);
                  }
	      
                  dBoxTouchesBox(newp1, dBodyGetRotation(l1->id), l1->sides, 
                  newp2,  dBodyGetRotation(l2->id), l2->sides);
		      
	      
                  l1->limbdamage = maxdepth;
                  l2->limbdamage = maxdepth;
                */

            }

	  #ifdef DRAWSTUFF
            if (VISUAL)
            {
                // draw little red cubes at contact points
                if (numc > 0) {
                    for (int i=0; i<numc; i++) {
                        dsSetColor(1,0,0);
                        //	      dsDrawBox (contact[i].geom.pos,IDENTITY,ss);
                    }
                }
            }
#endif
        }
    }
}

void doWorld (int pause, dReal step, int fast)
{

    // perform a cycle of the simulation.
    
    
    // initialisations...
    int noreg=1;
    for (int i=0; i < REGISTSIZE; i++)
    {
        if (regist[i])
            for (int l=0; l < regist[i]->nblimbs(); l++)
            {
                regist[i]->limbs[l].curdamage = 0;
                regist[i]->limbs[l].touchesOther = 0;
            }
        for (int j=0; j < REGISTSIZE; j++)
        {
            DAMAGETABLE[i][j] = 0;
        }
    }

    // collision detection
    dSpaceCollide (globalspace,0,&nearCallback);

    if (!pause)  
    {
        for (int i=0; i < REGISTSIZE; i++)
        {
            if (regist[i])
            {
                noreg = 0;
                regist[i]->findClosestAnimat();
                for (int l=0; l < regist[i]->nblimbs(); l++)
                {
                    if (WORLDTYPE == SPHERICWORLD)
                    {
                        // add a gravitation force
                        dBodyAddForce(regist[i]->limbs[l].id, 
                                      - GRAVCORR * dBodyGetPosition(regist[i]->limbs[l].id)[0],
                                      - GRAVCORR * dBodyGetPosition(regist[i]->limbs[l].id)[1],
                                      - GRAVCORR * dBodyGetPosition(regist[i]->limbs[l].id)[2]
                            );
                    }
		    
                    if (regist[i]->limbs[l].immunitytimer)
                        regist[i]->limbs[l].immunitytimer -- ;
		    
                    // update some tracing data
                    for (int n =0; n < 12; n++)
                        regist[i]->limbs[l].oldrot[n] =
                            dBodyGetRotation(regist[i]->limbs[l].id)[n];
                    for (int n=0; n < 3; n++)
                        regist[i]->limbs[l].oldpos[n] = 
                            dBodyGetPosition(regist[i]->limbs[l].id)[n];
                }

                // perform a neural update cycle
                if (regist[i]->alive) 
                    regist[i]->actuate();
                else 
                { 
                    mydie(
                        "Damn ! Animat registered but not alive in doWorld !\n");
                }
                /*if (VISUAL)
                  myprintf("Output neuron 3 limb 0: %f\n",
                  regist[i]->repres[0].neurons[3].out);*/
            }
        }
	
        if (fast)
            dWorldQuickStep(world, step); // quick, default
        else
            dWorldStep (world,step); // slower, more precise
        tot_time++;
    }
    dJointGroupEmpty (contactgroup);
}

#ifdef DRAWSTUFF
void drawSlant()
{
	dVector3 sides = {.1,20,20}; // vertical 'slice'
	dVector3 centre = {0, 0, 0};
	dMatrix3 rot;
	dRFromAxisAndAngle(rot, 0, 1, 0, -(M_PI / 4.0));
    dsSetColor(1,0,1);
	dsDrawBox(centre, rot, sides);
}
#endif
    
#ifdef DRAWSTUFF
static void simLoop (int pause)
{

    // this is the loop function to be called when performing graphical
    // simulations (see readbest.cpp and visual.cpp for examples)
    //
    // It draws everything and calls doWorld()
    
    int i, j;
    i = 0;
    static dReal p[3] = {0, 0, 0};
    dsSetColor (0,1,0);
    if (WORLDTYPE == SPHERICWORLD)
        dsDrawSphere(p, IDENTITY, WORLDRADIUS);
    dsSetColor (1,1,0);
    for (i=0; i < REGISTSIZE; i++)
        if (regist[i])
        {
            for (j=0; j < regist[i]->nblimbs(); j++)
            {
                dReal greenval;
                greenval =  (dReal)(HURTIMMUNITYTIMER - 
                                    regist[i]->limbs[j].immunitytimer)
                    / (dReal) HURTIMMUNITYTIMER;
                if (greenval < 0) greenval = 0;
                dsSetColor(1, greenval, 0);
                dsDrawBox (dBodyGetPosition(regist[i]->limbs[j].id), 
                           dBodyGetRotation(regist[i]->limbs[j].id),
                           regist[i]->limbs[j].sides);
            }
        }

    if (WALLS) drawWalls();
    if (CORRIDOR) drawCorridor();
    if (BALL)  ball.draw();
    doWorld(pause, STEP, true);
}
#endif

void resetScene()
{
    // reset the world 
    int i;
    for (i=0; i < REGISTSIZE; i++)
    {
        if (regist[i])
        {
            if (regist[i]->alive) 
                regist[i]->remove();
            else
                mydie("Damn ! Registered animat not alive !\n");
        }
        regist[i] = NULL;
    }
    dRandSetSeed(1); // dRandSetSeed sets the ODE-specific random seed
    if (BALL) 
    {
        ball.remove();
        ball.generate();
    }
}

void destroyWorld()
{
    dWorldDestroy (world);
    dJointGroupDestroy (contactgroup);
    dSpaceDestroy (globalspace);
}

void initWorld()
{

    // create the world
    //
    // must always be called at the start of a program
    
    int i;
    FILE *randfile;
    unsigned int seed;
    for (i=0; i < REGISTSIZE; i++)
        regist[i] = NULL;
    //srandom(MYRANDSEED);
    
    if (MYRANDSEED == 0)
    {
        randfile = fopen("/dev/urandom", "r");
        fread (&seed, sizeof(seed), 1, randfile);
        fclose(randfile);
    }
    else seed = MYRANDSEED;
    srandom(seed);
    myprintf("Random seed is %u\n", seed);
	
    #ifdef DRAWSTUFF
    // setup pointers to drawstuff callback functions
    fn.version = DS_VERSION;
    fn.start = &start;
    fn.step = &simLoop;
    fn.command = &command;
    fn.stop = 0;

    // NOTE: If you ARE using the modified drawstuff.cpp file, uncomment this
    // to prevent the ground from being drawn.

    //dsSetDrawGround(0);
    //dsSetShadows(0);

    // NOTE: If you ARE NOT using the modified drawstuff.cpp file, you will
    // need to set this directory to wherever the texture files are. 
    fn.path_to_textures = ".";
    //fn.path_to_textures = "/home/pg/txm/ode-0.5/drawstuff/textures";
    dsSetSphereQuality(3);
#endif
    
    // create world
    world = dWorldCreate();
    globalspace = dSimpleSpaceCreate (0); 
    alternatespace = dSimpleSpaceCreate (0);
    contactgroup = dJointGroupCreate (0);
    dRSetIdentity(IDENTITY);
    dWorldSetCFM (world, MYCFM);//0.05 // was 1e-5 - too low // .1 bad
    dWorldSetERP (world, MYERP); // should NOT be 0.5+ (too springy)*/
    tot_time = 0;
    numround = 0;
    if (WORLDTYPE == SPHERICWORLD)
    {
        dWorldSetGravity (world,0,0,0);
        sphereID = dCreateSphere (globalspace, WORLDRADIUS);
        groundID = NULL;
        dGeomSetPosition(sphereID, 0, 0, 0);
    }
    else if (WORLDTYPE == FLATWORLD)
    {
        dWorldSetGravity (world,0,0,-3);
        groundID = dCreatePlane (globalspace,0,0,1,0);
        sphereID = NULL;
    }
    else {
        myprintf("You must specify whether the world is spherical or flat!\n");
        myprintf("Please set the value of WORLDTYPE to SPHERICWORLD or FLATWORLD before calling initWorld()\n");
        mydie("Damn! World is neither spheric nor flat!\n");
    }
   
    if (BOARD) makeBoard();
    if (WALLS) makeWalls();
    if (CORRIDOR) makeCorridor();
    //if (BALL) ball.generate();
    // propriete des surfaces de contact des boites (voir test_boxstack.cpp)
    for (i=0; i< MAXCONT; i++) {
//    contact[i].surface.mode = dContactApprox1|dContactSoftCFM;
//    contact[i].surface.mode = dContactApprox1;
        contact[i].surface.mu = 1.0;
//    contact[i].surface.soft_cfm = 0.01;
    }

    
}
    
