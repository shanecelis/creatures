/* ----------------------------------------------------------------------------
  ex1.C
  mbwall 28jul94
  Copyright (c) 1995-1996  Massachusetts Institute of Technology

 DESCRIPTION:
   Example program for the SimpleGA class and 2DBinaryStringGenome class.
This program tries to fill the 2Dgenome with alternating 1s and 0s. 
  This example uses the default crossover (single point), default mutator
(uniform random bit flip), and default initializer (uniform random) for the
2D genome.
  Notice that one-point crossover is not necessarily the best kind of crossover
to use if you want to generate a 'good' genome with this kind of objective 
function.  But it does work.
---------------------------------------------------------------------------- */
#include <ga/GASimpleGA.h>	// we're going to use the simple GA
#include <ga/GA2DBinStrGenome.h> // and the 2D binary string genome
#include <ga/GAStatistics.h> 
#include <ga/std_stream.h>
#include <sys/stat.h>
#include <string>
#include <sstream>
#include <iostream>
#include "AnimatGenome.h"
#include "animat_eval.h"
#define cout STD_COUT

int my_mkdir(char* name) {
    return mkdir(name, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
}

int
main(int argc, char **argv)
{

// See if we've been given a seed to use (for testing purposes).  When you
// specify a random seed, the evolution will be exactly the same each time
// you use that seed number.

  for(int ii=1; ii<argc; ii++) {
    if(strcmp(argv[ii++],"seed") == 0) {
      GARandomSeed((unsigned int)atoi(argv[ii]));
    }
  }

// Declare variables for the GA parameters and set them to some default values.

  int popsize  = 300;
  int ngen     = 100;
  float pmut   = 0.000;
  float pcross = 0.0;

// Now create the GA and run it.  First we create a genome of the type that
// we want to use in the GA.  The ga doesn't operate on this genome in the
// optimization - it just uses it to clone a population of genomes.

  WORLDTYPE = FLATWORLD;  // let's go all presocratic here...
  initWorld();


  AnimatGenome genome;

// Now that we have the genome, we create the genetic algorithm and set
// its parameters - number of generations, mutation probability, and crossover
// probability.  And finally we tell it to evolve itself.

  
  GASimpleGA ga(genome);
  ga.populationSize(popsize);
  ga.nGenerations(ngen);
  ga.pMutation(pmut);
  ga.pCrossover(pcross);
  ga.scoreFrequency(1);
  ga.flushFrequency(1);
  ga.selectScores(GAStatistics::AllScores);
  ga.scoreFilename("scores.data");
  ga.selector(GARouletteWheelSelector());
  ga.scaling(GANoScaling());
  //ga.evolve();
  for (int i = 0;! ga.done(); i++) {
      char filename[255];
      sprintf(filename, "pop%.3d.data", i);
      //ofstream f(filename);
      const GAPopulation& pop = ga.population();
      pop.sort();
      // Don't write the population.
      //f << pop;
      char dirname[255];
      sprintf(dirname, "dpop%.3d", i);
      my_mkdir(dirname);
      for (int j = 0; j < popsize; j++) {
          char indfile[255];
          sprintf(indfile, "%s/ind%.3d.gen", dirname, j);
          ofstream ind(indfile);
          if (ind) {
              ind << pop.best(j);
              ind.close();
          } else {
              cerr << "error: Unable to write to " << indfile << "\n";
          }
          AnimatGenome& g = (AnimatGenome&) pop.best(j);
          sprintf(indfile, "%s/ind%.3d.json", dirname, j);
          g.animat->save(indfile);
          sprintf(indfile, "%s/ind%.3d.bin", dirname, j);
          g.animat->saveOld(indfile);
      }
      ga.step();

  }
  //ga.write("ex1.pop");
  //cout << ga.statistics().bestIndividual();
  //cout << ga.population();

// Now we print out the best genome that the GA found.

  //cout << "The GA found:\n" << ga.statistics().bestIndividual() << "\n";

  //((GAStatistics&)ga.statistics()).scores(cout, GAStatistics::AllScores);

// That's it!
  return 0;
}
 



// This is the objective function.  All it does is check for alternating 0s and
// 1s.  If the gene is odd and contains a 1, the fitness is incremented by 1.
// If the gene is even and contains a 0, the fitness is incremented by 1.  No
// penalties are assigned. 
//   We have to do the cast because a plain, generic GAGenome doesn't have 
// the members that a GA2DBinaryStringGenome has.  And it's ok to cast it
// because we know that we will only get GA2DBinaryStringGenomes and
// nothing else.

