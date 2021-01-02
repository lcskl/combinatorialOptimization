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


#endif