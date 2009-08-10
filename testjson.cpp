#include "animat.h"

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


int main(int argc, char **argv) {
    Animat A;
    Animat B;

    Animat animats[3];
    //A.randGenome();
    //A.save(argv[1]);
    /*
    if (1 || argc == 3) {
        B.read(argv[1]);
        B.save(argv[2]);
        }*/
    animats[0].fitness = 2;
    animats[1].fitness = 3;
    animats[2].fitness = 2.5;
    qsort(animats, 3, sizeof(Animat), descendFitness);
    for (int i = 0; i < 3; i++) {
        printf("fitness %f\n", animats[i].fitness);
    }
}
