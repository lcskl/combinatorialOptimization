#include <iostream>
#include <cstdlib>
#include <chrono>

#include "graph.hpp"
#include "match_algo.hpp"


int main(int argc, char** argv)
{
   std::ios_base::sync_with_stdio(false);
   if (argc != 2)
   {
      std::cerr << "Wrong number of arguments. Program call: <program_name> <input_graph>" << std::endl;
      return EXIT_FAILURE;
   }

   ED::Graph graph = ED::Graph::build_graph(argv[1]);
   Matching M = max_matching(graph, argv[1]);

   M.print(graph.num_nodes());

   return EXIT_SUCCESS;
}
