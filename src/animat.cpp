#include "animat.h"
#include "stdarg.h"
// First, a few utility function for displaying messages or saving them to
// disk...

// SENSORTYPES defines which sensors are to be used in the simulation (besides
// proprioceptors, which are always used).  MODIFY THIS (and also
// NBUSEDSENSORTYPES) if you want to use more or fewer sensors in your
// simulation - of course, don't forget to update fillSensors as well.

//#define NBSENSORTYPES 7
//int SENSORTYPES[NBSENSORTYPES] = {SENSBALLX, SENSBALLY, SENSCLOSESTANIMX,
//                                SENSCLOSESTANIMY, SENSTOUCH, SENSUPDOWN, SENSLEFTRIGHT };

//int SENSORTYPES[NBSENSORTYPES] = { SENSGOSTOP };
int SENSORTYPES[NBSENSORTYPES] = { SENSUPDOWN };

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

dReal GRAVCORR = 4.0 / (dReal) WORLDRADIUS;
dWorldID world;
dMatrix3 IDENTITY;
dSpaceID globalspace;
dSpaceID alternatespace; // only used within collision method

/*
  upDown and leftRight are global variables that describe how the
  creature is supposed to be moving.
 */
double goStop = 0.0f; 
double upDown = 0.0f; 
double leftRight = 0.0f;

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
int board [BSIDE][BSIDE];
Animat *regist[REGISTSIZE];
Ball ball;




//int OUTPUTREDIRECT = TONULL;
int OUTPUTREDIRECT = TOSTDOUT;

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
    throw "exit";
    exit(i);
}


// exits program, write reason to "exitmessage.txt"
void mydie(const char *msg, ...)
{
    va_list argp;
    FILE *out;
    va_start(argp, msg);
    vprintf(msg, argp);
    va_end(argp);
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

int isExtSensor(Neuron *N)
{
    for (int i=0; i < NBSENSORTYPES; i++)
        if (N->type == SENSORTYPES[i])
            return 1;
    return 0;
}

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
            if (repres[i].neurons[neur].type == SENSGOSTOP)
            {
                repres[i].neurons[neur].out = goStop;
                if (DISABLESENSORS)
                    repres[i].neurons[neur].out = 0;
                //myprintf("go stop neuron %.6f\n", repres[i].neurons[neur].out);
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


void airCallback (void *data, dGeomID o1, dGeomID o2)
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
void nearCallback (void *data, dGeomID o1, dGeomID o2)
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

void doWorld (int pause, dReal step, int fast, int useBrains)
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
                if (regist[i]->alive) {
                    if (useBrains) {
                        regist[i]->actuate();
                    } else {
                        for (int j = 1; j < regist[i]->nblimbs(); j++) {
                            dJointSetHingeParam(regist[i]->limbs[j].joint, dParamFMax, 0);
                        }
                    }
                } 
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
void simLoop (int pause)
{
    simLoopPlus(pause, true);
}

void simLoopPlus (int pause, int useBrains)
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
    doWorld(pause, STEP, true, useBrains);
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
    dInitODE();
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
