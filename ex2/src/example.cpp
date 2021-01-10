#include <iostream> // For writing to the standard output.
#include <fstream> // For reading input files.

#include "graph.hpp"
#include "join.hpp"
#include "utils.hpp"
#include "../blossom5-v2.05.src/PerfectMatching.h"

int main(int argc, char** argv)
{
   // If you don't want to write MMC:: all the time,
   // using a namespace in the implementation of some function is perfectly ok.
   using namespace MMC;

   if (argc != 3)
   {
      std::cout << "Expected exactly two arguments (path to input graph and path to output graph)!" << std::endl;
      return EXIT_FAILURE; // return 1 would do the same, but is way too easy to mix up!
   }
   std::fstream input_file{argv[1]};
   if (input_file.fail())
   {
      std::cout<<"Failed to open the input file. Exiting."<<std::endl;
      return EXIT_FAILURE;
   }
   std::ofstream output_file(argv[2], std::ios::out | std::ios::trunc);
   if (output_file.fail())
   {
      std::cout<<"Failed to open the output file. Exiting."<<std::endl;
      return EXIT_FAILURE;
   }
   Graph const graph = Graph::read_dimacs(input_file);

   //We are currently checking this to make sure that the graph we call the minimum weight perfect matching
   //solver on possesses a perfect matching.
   //As you won't call the solver on the input graph directly,
   // but on the disjoint union of complete graphs on the sets of
   //odd(E^-) vertices in the connected components of G, you should remove this check.
   if (graph.num_edges() == graph.num_nodes() * (graph.num_nodes() - 1))
   {
      std::cout << "Expected input graph to be complete graph!" << std::endl;
      return EXIT_FAILURE;
   }
   else if (graph.num_nodes() % 2 != 0)
   {
      std::cout << "Expected input graph to have even number of vertices!" << std::endl;
      return EXIT_FAILURE;
   }
   else
   {
      minimum_weight_empty_join(graph);

      return EXIT_SUCCESS; 
   }
}

