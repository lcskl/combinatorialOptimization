#ifndef UTILS_HPP
#define UTILS_HPP

/**
   @file utils.hpp

   @brief This file provides methods that are used by the main algorithm
**/

#include "graph.hpp"
#include <vector>

/**
 * Simple DFS that stamps each NodeId with a specific value (to easily identify connected components)
 *
 * @param G Original graph
 * @param vertex NodeId of the current vertex in the search
 * @param visited Array of stamps (visit a vertex only once)
 * @param stamp Value that will be given to all vertices in current connected component
 */
void dfs(const MMC::Graph & G,MMC::NodeId vertex, std::vector<int> & visited,int stamp);

/**
 * Floyd and Warshall's Algorithm implementation with back-tracking
 * @param G Graph whose min. weight paths are to be calculated
 * @return Pair of vectors: (Min weight values matrix in array form -- Matrix (in array form) of last visited vertex in a min path -> backtracking)
 */
std::pair<std::vector<MMC::EdgeWeight>,std::vector<MMC::HalfEdgeId>> floyd_warshall(const MMC::Graph & G);

/**
 * Simply returns a copy of the graph with absolute valued weights
 *
 * @param G Input Graph with cost function c(e)
 * @return Graph G' with cost function d(e) = |c(e)| 
 */
MMC::Graph copy_abs_weight(const MMC::Graph & G);

/**
 * Simply returns a copy of the graph with absolute valued weights
 *
 * @param G Input Graph whose cost function is to be updated
 * @param value Real value to be summed to each edge weight
 */
void reduce_weights(MMC::Graph & g,const double value);

#endif