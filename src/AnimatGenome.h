#ifndef _ANIMATGENOME_H_
#define _ANIMATGENOME_H_

#include <ga/gaid.h>
#include <ga/gaconfig.h>
#include <ga/gaerror.h>
#include <ga/GAEvalData.h>
#include <ga/std_stream.h>
#include <ga/GAGenome.h>
#include <ga/garandom.h>
#include "animat.h"
#include "animat_eval.h"

using namespace std;
// Class definition for the new genome object, including statically defined
// declarations for default evaluation, initialization, mutation, and 
// comparison methods for this genome class.
class AnimatGenome : public GAGenome {
public:
    GADefineIdentity("AnimatGenome", 201);
    static void Init(GAGenome&);
    static int Mutate(GAGenome&, float);
    static float Compare(const GAGenome&, const GAGenome&);
    //static float Evaluate(GAGenome&);
    static float fourway(GAGenome&);
    static float fourwayCont(GAGenome&);
    static float run(GAGenome&);
    static float run2(GAGenome&);
    static int Cross(const GAGenome&, const GAGenome&, GAGenome*, GAGenome*);

public:

    Animat *animat;
AnimatGenome(Evaluator eval = fourwayCont) : GAGenome(Init, Mutate, Compare) { 
        animat = new Animat();
        Init(*this);
        evaluator(eval); 
        crossover(Cross); 
    }
    AnimatGenome(const AnimatGenome& orig) { copy(orig); }
    virtual ~AnimatGenome() {}
    AnimatGenome& operator=(const GAGenome& orig){
        if(&orig != this) copy(orig);
        return *this;
    }

    virtual GAGenome* clone(CloneMethod) const {return new AnimatGenome(*this);}
    virtual void copy(const GAGenome& orig) {
        GAGenome::copy(orig);  // this copies all of the base genome parts
        // copy any parts of MyObject here
        // copy any parts of AnimatGenome here
        animat = new Animat();
        animat->copyFrom(((AnimatGenome&)orig).animat);
    }

#ifdef GALIB_USE_STREAMS
    virtual int read(STD_ISTREAM &);
    virtual int write(STD_OSTREAM &ostr)const ;
#endif

// any data/member functions specific to this new class
};

void 
AnimatGenome::Init(GAGenome& a){
    // your initializer here
    //cout << "INIT!" << endl;
    AnimatGenome& g = (AnimatGenome&) a;
    if (g.animat) {
        delete g.animat;
    }
    g.animat = new Animat();
    g.animat->randGenome();
    int wrong = 0;
    int count = 0;
    do { 
        g.animat->mutate();
        g.animat->reassignBadConns();
        g.animat->checkGenome();
        g.animat->checkConnections();
        
        wrong = isColliding(g.animat);
        count++;
    } while (wrong && count < 100);
}

int 
AnimatGenome::Mutate(GAGenome& a, float p){
    // your mutator here
    //cout << "MUTATE!\n";
    AnimatGenome& g = (AnimatGenome&) a;
    g._evaluated = gaFalse;
    
    int wrong = 0;
    int count = 0;
    do { 
        g.animat->mutate();
        g.animat->reassignBadConns();
        g.animat->checkGenome();
        g.animat->checkConnections();
        
        wrong = isColliding(g.animat);
        count++;
    } while (wrong && count < 100);
    //return ! wrong;
    return count;
}

float 
AnimatGenome::Compare(const GAGenome&, const GAGenome&){
    // your comparison here
    return -1.0;
}

/* float  */
/* AnimatGenome::Evaluate(GAGenome&){ */
/*     // your evaluation here */
/* } */

float
AnimatGenome::fourway(GAGenome& a) {
    AnimatGenome& g = (AnimatGenome&) a;
    return (float) eval_fourway(g.animat);
}

float
AnimatGenome::run(GAGenome& a) {
    //return GARandomFloat(0.0, 10.0);
    AnimatGenome& g = (AnimatGenome&) a;
    return (float) prim_eval(g.animat);
}

float
AnimatGenome::run2(GAGenome& a) {
    //return GARandomFloat(0.0, 10.0);
    AnimatGenome& g = (AnimatGenome&) a;
    DistanceProcess d;
    eval_callback(g.animat, &d);
    return d.distance();
}

class FourwayContProcess : public Process {
public:
    dReal beginPos[3];
    dReal endPos[3];
    dReal rup[3];
    dReal rdown[3];
    dReal rstop[3];
    dReal rleft[3];
    dReal rright[3];
    dReal *values[5];
    int sections;
    int delta;
    int section;
    FourwayContProcess() {
        values[0] = rstop;
        values[1] = rup;
        values[2] = rleft;
        values[3] = rdown;
        values[4] = rright;
        sections = 5;
        section = 0;
    }
    
    void setNeurons(int section) {
        cerr << "debug: setting neurons for section " << section << "\n";
        switch (section) {
        case 0: upDown = 0.0; leftRight = 0.0; break;
        case 1: upDown = 1.0; leftRight = 0.0; break;
        case 2: upDown = 0.0; leftRight = -1.0; break;
        case 3: upDown = -1.0; leftRight = 0.0; break;
        case 4: upDown = 0.0; leftRight = 1.0; break;
        default:
            cerr << "error: setNeurons got section " << section << "\n";
        }
    }

    int timestep(int timestep, int lastTimestep, Animat *A)
    {
        dReal *pos;
        if (timestep == 0) {
            delta = lastTimestep/sections;
        } 
        if (timestep == lastTimestep + 1) {
            cout << "fourwayContFitness: " << fitness() << "\n";
            return 0;
        } else if (timestep > lastTimestep + 1) {
            return 0;
        }
        
        if (timestep == (section + 1) * delta || timestep == lastTimestep) {
            // End of this section
            pos = A->getAvgPos();
            vset(endPos, pos);
            sub(beginPos, endPos, values[section]);
            myprintf("Section %d: {%f, %f, %f}\n", section, values[section][0], 
                     values[section][1], 
                     values[section][2]);
                    

            section++;
        }

        if (timestep == section * delta && section != sections) {
            // Beginning of this section
            setNeurons(section);
            pos = A->getAvgPos();
            vset(beginPos, pos);
        }  

    }

    float fitness() {
        // Let's project everything into the x-y plane at z = 0.
        rup[2] = rdown[2] = rstop[2] = rleft[2] = rright[2] = 0.0;
        
        // Let a be the angle between rup and y positive, (0,1,0).  
        double a = acos(rup[1]/norm2(rup));
        dReal n[3] = {0.0,0.0,0.0};
        double fitness = fitness_part(rstop, n);
        
        normalize(rup, n);
        fitness *= fitness_part(rup, n);
        rotate(n, M_PI_2, n);
        fitness *= fitness_part(rleft, n);
        rotate(n, M_PI_2, n);
        fitness *= fitness_part(rdown, n);
        rotate(n, M_PI_2, n);
        fitness *= fitness_part(rright, n);
        
        dReal amean[3], std[3];

        mean(5, values, amean);
        stddev(5, values, amean, std);
        fitness *= (norm2(amean) + norm2(std));
        myprintf("Eval_fourwayCont: %f\n", fitness);
        return fitness;
    }
};


float
// four way continuous.
AnimatGenome::fourwayCont(GAGenome& a) {
    //return GARandomFloat(0.0, 10.0);
    AnimatGenome& g = (AnimatGenome&) a;
    FourwayContProcess d;
    eval_callback(g.animat, &d);
    return d.fitness();
}

int
AnimatGenome::Cross(const GAGenome& mom, const GAGenome& dad,
                    GAGenome* sis, GAGenome* bro){
    // your crossover here
    /* Individ_Animat* other = (Individ_Animat *) ind; */
    /* int wrong = 0; */
    /* animat->crossWith(other->animat); */
    /* int count = 0; */
    /* do {  */
    /*     animat->mutate(); */
    /*     animat->reassignBadConns(); */
    /*     animat->checkGenome(); */
    /*     animat->checkConnections(); */
            
    /*     wrong = isColliding(animat); */
    /*     count++; */
    /* } while (wrong && count < 100); */
    /* return ! wrong; */
    return 0;
}

#ifdef GALIB_USE_STREAMS
int AnimatGenome::read(STD_ISTREAM &in)
{ 
    char buf[50000];
    in.getline(buf, 49999);
    float fitness;
    fitness = atof(buf);
    cerr << "read fitness " << fitness << "\n";
    in.getline(buf, 49999);

    char filename [L_tmpnam];
    tmpnam(filename);
    std::ofstream out(filename);
    out << buf;
    out.close();
    this->animat->read(filename);
    unlink(filename);
    return 0;
}
int AnimatGenome::write(STD_OSTREAM &ostr) const
{ 
    
    char filename [L_tmpnam];
    tmpnam(filename);
    this->animat->save(filename);
    //cerr << "write individual to file " << filename << endl; 
    std::ifstream ifs(filename);
    if (ifs.good()) {
        //std::stringstream oss;
        ostr << this->evaluate() << "\n"; // write the fitness
        ostr << ifs.rdbuf();
    } else {
        cerr << "Unable to read from temp file "<< filename <<".\n";
        return 1;
    }
    unlink(filename);
    return 0;
}
#endif

#endif /* _ANIMATGENOME_H_ */
