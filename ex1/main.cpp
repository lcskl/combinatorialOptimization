#include <iostream>
#include <cstdlib>

#include "graph.hpp"
#include "match_algo.hpp"


int main(int argc, char** argv)
{
   if (argc != 2)
   {
      std::cerr << "Wrong number of arguments. Program call: <program_name> <input_graph>" << std::endl;
      return EXIT_FAILURE;
   }

   ED::Graph graph = ED::Graph::build_graph(argv[1]);
   Matching M = bipartite_perfect_matching(graph);
   M.print();
   std::cout << graph;
   return EXIT_SUCCESS;
}
