#include <iostream> // For writing to the standard output.
#include <fstream> // For reading input files.

#include "graph.hpp"
#include "join.hpp"
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
      PerfectMatching solver{
	 static_cast<int>(graph.num_nodes()),
         static_cast<int>(graph.num_edges())};
      for (EdgeId edge_id{0}; edge_id < graph.num_edges(); ++edge_id)
      {
         solver.AddEdge(
	   graph.halfedge(2*edge_id + 1).target(),
	   graph.halfedge(2*edge_id + 0).target(),
	   graph.edge_weight(edge_id));
      }
 
      solver.Solve();

      Graph solution{graph.num_nodes()};
      for (EdgeId edge_id{0}; edge_id < graph.num_edges(); ++edge_id)
      {
        if (solver.GetSolution(edge_id))
         {
               solution.add_edge(
            graph.halfedge(2*edge_id + 1).target(),
            graph.halfedge(2*edge_id + 0).target(),
            graph.edge_weight(edge_id));
         }
      }
      output_file << solution;


      return EXIT_SUCCESS; 
   }

}

