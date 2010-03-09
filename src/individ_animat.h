#include <alps_individ.h>
#include "animat.h"
#include <iostream>
using namespace alps;
using namespace std;

static int save_count = 0;
class Individ_Animat : public Individual {
    public:
    Animat *animat;
    Individ_Animat() {
        //cerr << "constructed" << endl;
        animat = NULL;
    };
    virtual ~Individ_Animat() {
        if (animat) {
            delete animat;
            animat = NULL;
        }
    };
    void initialize() {
        //cerr << "initialize" << endl;
        Individual::initialize();
        animat = new Animat();
    };

/*     bool keep(bool maximize) { */
/*         return valid(); */
/*     } */

    Individual* new_instance() {
        return new Individ_Animat();
    }
    bool valid() {
        try {
            animat->checkGenome();
            animat->checkConnections();
            return ! isColliding(animat);
        } catch (...) {
            return false;
        }
    }
    
    void make_random() {
        //cerr << "random" << endl;
        Individual::make_random();
        animat->randGenome();
        int wrong = 0;
        int count = 0;
        do { 
            animat->mutate();
            animat->reassignBadConns();
            animat->checkGenome();
            animat->checkConnections();
            
            wrong = isColliding(animat);
            count++;
        } while (wrong && count < 100);
        //return ! wrong;
    };
    
    void duplicate_settings(Individual *ind) {        
        cerr << "duplicate_settings" << endl;
        Individual::duplicate_settings(ind);
    };

    void duplicate(Individual *ind) {
        //cerr << "duplicate" << endl;
        duplicate_settings(ind);
        Individual::duplicate(ind);
        Individ_Animat* other = (Individ_Animat *) ind;
        animat->copyFrom(other->animat);
    };
    
    bool mutate() {
        //cerr << "mutate" << endl;
        Individual::mutate();
        int wrong = 0;
        int count = 0;
        do { 
            animat->mutate();
            animat->reassignBadConns();
            animat->checkGenome();
            animat->checkConnections();
            
            wrong = isColliding(animat);
            count++;
        } while (wrong && count < 100);
        return ! wrong;
    };
    
    bool recombine(Individual *ind) {
        //cerr << "recombine" << endl;
        Individual::recombine(ind);
        Individ_Animat* other = (Individ_Animat *) ind;
        int wrong = 0;
        animat->crossWith(other->animat);
        int count = 0;
        do { 
            animat->mutate();
            animat->reassignBadConns();
            animat->checkGenome();
            animat->checkConnections();
            
            wrong = isColliding(animat);
            count++;
        } while (wrong && count < 100);
        return ! wrong;
    };

    bool recombine_rand2(Individual *ind) {
        return recombine(ind);
    }

/*     //int phenotype_distance(Individual* ind2); */
/*     bool make_phenotype(void* arg); */
/*     bool make_phenotype(); */
/*     std::istream& read(std::istream& istr); */
    std::ostream& write(std::ostream& ostr) {
        /* char filename[255]; */
        /* double fitness = Individual::get_fitness(); */
        /* sprintf(filename, "best-%.3d.json", save_count); */
        /* cerr << "write individual with fitness " << fitness << " to file " << filename << endl; */
        /* save_count++; */
        /* animat->save(filename); */

        double fitness = Individual::get_fitness(); 
        // Write it to a temporary file first, then read that and
        // output it to the stream.  (C++ and C interacting, badly.)
        char filename [L_tmpnam];
        tmpnam(filename);
        animat->save(filename);
        cerr << "write individual with fitness " << fitness << " to file " << filename << endl; 
        std::fstream ifs(filename);
        if (!ifs) {
            //std::stringstream oss;
            ostr << ifs.rdbuf();
        }
        return ostr;
    }

};
