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

   minimum_weight_empty_join(graph);

}

