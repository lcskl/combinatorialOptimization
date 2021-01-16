#include <iostream> // For writing to the standard output.
#include <fstream> // For reading input files.
#include <limits>

#include "graph.hpp"
#include "join.hpp"
#include "utils.hpp"
#include "../blossom5-v2.05.src/PerfectMatching.h"

#include "min_mean_cycle.hpp"

#include "checker.hpp"

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
   Graph graph = Graph::read_dimacs(input_file);

   std::vector<EdgeWeight> initialWeights(graph.num_edges());  //save them because they get modified
   for(size_t i = 0; i < graph.num_edges(); ++i ) initialWeights[i] = graph.edge_weight(i);    //TODO better, save in output file

   //auto r = min_mean_cycle(graph);

   std::vector<NodeId> C = minimum_mean_weight_cycle(graph);

   

   //double value = 0.0;

   output_file << "p edge " << graph.num_nodes() << " " << C.size() << std::endl;
   for(size_t i = 0; i < C.size(); ++i){
       //value += initialWeights[C[i]];
       output_file << "e " << graph.halfedge(2*C[i]).target()+1 << " " << graph.halfedge(2*C[i]+1).target()+1 << " " << initialWeights[C[i]] << std::endl;
   }

   //std::cout << "ALGO: " << value/(double)C.size() << std::endl;
   //std::cout << "BF: " << r.second << std::endl;
}

