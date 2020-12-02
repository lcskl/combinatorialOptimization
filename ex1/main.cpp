#include <iostream>
#include <cstdlib>

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
   Matching M = perfect_matching(graph);
   std::cout<< "\n\nVeredict:\n";
   if(M._perfect == false)
      std::cout << "No Perfect Matching Found!\n";
   else{ 
      std::cout << "Perfect Matching:\n";
      M.print();
   }
   //std::cout << graph;
   return EXIT_SUCCESS;
}
