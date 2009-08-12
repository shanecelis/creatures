#include <alps_individ.h>
#include "animat.h"
#include <iostream>
using namespace alps;
using namespace std;

class Individ_Animat : public Individual {
    public:
    Animat *animat;
    Individ_Animat() {
        cerr << "constructed" << endl;
        animat = NULL;
    };
    virtual ~Individ_Animat() {
        if (animat) {
            delete animat;
            animat = NULL;
        }
    };
    void initialize() {
        cerr << "init" << endl;
        animat = new Animat();
    };

    bool keep(bool maximize) {
        return true;
    }

    Individual* new_instance() {
        return new Individ_Animat();
    }
    bool valid() {
        return true;
    }
    
    void make_random() {
        cerr << "random" << endl;
        animat->randGenome();
    };
    
    void duplicate_settings(Individual *individ2) {        
        cerr << "duplicate_settings" << endl;
        Individual::duplicate_settings(individ2);
    };

    void duplicate(Individual *ind) {
        cerr << "duplicate" << endl;
        duplicate_settings(ind);
        Individ_Animat* other = (Individ_Animat *) ind;
        animat->copyFrom(other->animat);
    };
    
    bool mutate() {
        cerr << "mutate" << endl;
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
        cerr << "recombine" << endl;
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

/*     //int phenotype_distance(Individual* ind2); */
/*     bool make_phenotype(void* arg); */
/*     bool make_phenotype(); */
/*     std::istream& read(std::istream& istr); */
/*    std::ostream& write(std::ostream& ostr); */

};
