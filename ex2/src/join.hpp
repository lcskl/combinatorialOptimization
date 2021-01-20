#ifndef JOIN_HPP
#define JOIN_HPP

#include "graph.hpp"
/**
   @file join.hpp

   @brief This file provides the methods and functions to compute minimum weight empty joins
**/

using namespace MMC;

/**
 * Struct for representing a connected subgraph. 
*/
struct ConnectedComponent{
   Graph graph; //Actual Component
   std::vector<NodeId> originalNodeID; //Dictionary between this nodeId (from the subgraph) and the one from the underlying super-graph
};

/**
 * A complete graph and a matrix (in array form) representing the next half edge in a min path on the original graph 
*/
struct MetricClosure{
   Graph graph; //Actual complete subgraph
   std::vector<HalfEdgeId> next_in_min_path; //Next half edge in a min path
};

/**
 * Calculates the minimum weight empty-join of a graph (minimum weight eulerian subgraph)
 *
 * @param G Original graph, whose minimum 0-join is wanted
 * @return Set of Id's whose correspondent edges are on the 0-join (It might be empty)
 */
std::set<EdgeId> minimum_weight_empty_join(const Graph & G);

/**
 * Given a subset of vertices, it creates new 'Graph' objects for each connected component spanned by this subset.
 *
 * @param G Original graph
 * @param spanningSet Subset of vertices (NodeId's) of G whose induced subgraph is wanted
 * @return List of pairs of 'Graph' objects and a vector that can be used as dictionary between NodeId's in those new graphs
 * and the original graph.
 */
std::vector< ConnectedComponent > connected_components(const Graph & G,const std::vector<NodeId> & spanningSet);

/**
 * Given a Graph G, it computes its metric closure and the min x,y paths for every vertices x,y
 * @param G Original graph whose metric closure is wanted
 * @return MetricClosure struct (both the graph object itself plus a matrix for min path retrieval)
 */ 
MetricClosure metric_closure(const Graph & G);

#endif
