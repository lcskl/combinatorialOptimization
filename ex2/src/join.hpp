#ifndef JOIN_HPP
#define JOIN_HPP

#include "graph.hpp"
/**
   @file join.hpp

   @brief This file provides the methods and functions to compute minimum weight empty matchings
**/

using namespace MMC;

/**
 * Calculates the minimum weight empty-join of a graph (minimum weight eulerian subgraph)
 *
 * @param G Original graph, whose minimum 0-join is wanted
 * @return Set of Id's whose correspondent edges are on the 0-join
 */
std::set<EdgeId> minimum_weight_empty_join(const Graph & G);

/**
 * Given a subset of vertices, it creates new 'Graph' objects for each connected components spanned by this subset.
 *
 * @param G Original graph
 * @param spanningSet Subset of vertices (NodeId's) of G
 * @return List of pairs of 'Graph' objects and a vector that can be used as dictionary between NodeId's in those new graphs
 * and the original graph.
 */
std::vector< std::pair<Graph,std::vector<NodeId>> > connected_components(const Graph & G, std::vector<NodeId> spanningSet);

#endif
