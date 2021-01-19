#include "join.hpp"
#include "utils.hpp"
#include "../blossom5-v2.05.src/PerfectMatching.h"
#include <iostream>
#include <algorithm>
#include <set>
#include <map>
#include <cmath>
#include <chrono>

std::vector<std::pair<NodeId,NodeId>> perfect_matching(const Graph & graph){
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

    std::vector<std::pair<NodeId,NodeId>> matching;
    
    for (EdgeId edge_id{0}; edge_id < graph.num_edges(); ++edge_id)
    {
    if (solver.GetSolution(edge_id))
        {
            matching.push_back(std::make_pair(graph.halfedge(2*edge_id + 1).target(),
                                              graph.halfedge(2*edge_id + 0).target()));
        }
    }
    return matching;
}

std::pair <Graph,std::vector<NodeId>> metric_closure(const Graph & G){
    auto infty = std::numeric_limits<EdgeWeight>::max();
    std::vector<bool> edge_between(G.num_nodes()*G.num_nodes(), false);

    for(NodeId v=0 ; v < G.num_nodes(); v++ ){
        for(auto out_hf_id : G.node(v).outgoing_halfedges()){
            auto neighbor = G.halfedge(out_hf_id).target();
            edge_between[v*G.num_nodes()+neighbor] = true;
            edge_between[neighbor*G.num_nodes()+v] = true;
        }
    }

    auto FW_return = floyd_warshall(G);
    auto distances = FW_return.first;

    Graph G_met_cl(G.num_nodes());

    for(NodeId v=0 ; v < G.num_nodes(); v++ ){
        for(NodeId u=v+1 ; u < G.num_nodes(); u++ ){
            if(distances[v*G.num_nodes()+u] != infty)
                G_met_cl.add_edge(v,u,distances[v*G.num_nodes()+u]);
        }
    }

    return make_pair(G_met_cl,FW_return.second);
}

std::set<EdgeId> min_x_y_path(const NodeId x,const NodeId y, const std::vector<HalfEdgeId> & next, const Graph & G){
    std::set<EdgeId> path;
    if(next[x*G.num_nodes()+y] > 2*G.num_edges()){
        std::cout << x+1 << "--" << y+1 << " not connected\n";
        return path;
    }
    auto current_vertex = x;
    //std::cout << x << " ";
    while(current_vertex != y){
        path.insert(next[current_vertex*G.num_nodes()+y]/2); //In Graph Class, half edges have id's 2e and 2e+1 (e is the edge id - not directed edge)
        current_vertex = G.halfedge(next[current_vertex*G.num_nodes()+y]).target();
        //std::cout << current_vertex << " ";
    }
    //std::cout << "\n";
    return path;
}

std::set<EdgeId> minimum_weight_empty_join(const Graph & G){
    //This implementation is based on Theorems 51 and 52 of the lecture notes 
    //It computes a minimum empty join with real valued edge weights
    //According to Theorem 52, it suffices to calculate a V^- Join in G_d (weight function d(e) := |c(e)| )

    auto G_d = copy_abs_weight(G); //Copy of the original graph with with weight function d(e) := |c(e)|

    //std::cout << G_d << std::endl;

    auto t1 = std::chrono::high_resolution_clock::now();
    auto ret_closure = metric_closure(G_d); //Complete Graph with w({x,y}) = min_weight(x,y) in G_d 
    auto t2 = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();

    std::cout << duration << std::endl;

    auto G_closure = ret_closure.first; //List of distances
    auto next = ret_closure.second; //Next half edge in a min path

    auto T = G.odd_v_minus(); // Get V^-
    // for(auto x : T)
    //     std::cout << x+1 << " ";
    // std::cout << std::endl;
    auto conn_comps = connected_components(G_closure,T); //Components spanned by V^- 

    //std::cout << " Components of G(T):\n";
    std::set<EdgeId> path_sym_diff;
    for(auto comp : conn_comps){
        // std::cout << comp.first << std::endl;
        // for(NodeId x = 0; x < comp.second.size(); x++){
        //     std::cout << x+1 << " -> " << comp.second[x]+1 << std::endl; 
        // }

        auto perf_match = perfect_matching(comp.first);

        //Symmetric differences of edges in a min path from x to y 

        std::set<EdgeId> current;

        for(auto match_edge : perf_match){
            //std::cout << match_edge.first+1 << " --- " << match_edge.second+1 << std::endl;
            auto x = comp.second[match_edge.first];
            auto y = comp.second[match_edge.second];

            current = path_sym_diff;
            path_sym_diff.clear();

            auto path = min_x_y_path(x,y,next,G_d);
            // std::cout << "Min X-Y Path: \n";
            // for(auto edge_id : path){
            //     std::cout << "Edge Id: " << edge_id << " -> " << G_d.halfedge(2*edge_id).target()+1 << " -- " << G_d.halfedge(2*edge_id+1).target()+1 << std::endl;  
            // }
        
            std::set_symmetric_difference(path.begin(),path.end(),current.begin(),current.end(),std::inserter(path_sym_diff,path_sym_diff.end()));
        }
    }

    auto negative_edges = G.e_minus();

    std::set<EdgeId> empty_join;

    std::set_symmetric_difference(negative_edges.begin(),negative_edges.end(),path_sym_diff.begin(),path_sym_diff.end(),std::inserter(empty_join,empty_join.end()));

    // std::cout << "Empty Join: \n";
    // for(auto edge_id : empty_join){
    //     std::cout << "Edge Id: " << edge_id << " -> " << G.halfedge(2*edge_id).target()+1 << " -- " << G.halfedge(2*edge_id+1).target()+1 << std::endl; 
    // }

    return empty_join;
}

std::vector< std::pair<Graph,std::vector<NodeId>> > connected_components(const Graph & G, std::vector<NodeId> spanningSet){
    std::vector< std::pair<Graph,std::vector<NodeId>> > components;

    auto n = G.num_nodes();
    std::vector<int> visited(n,-1); //-1 for vertices not in SpanningSet, 0 for unvisited in spanningSet, otherwise STAMP of the connected component

    for(auto node : spanningSet){ visited[node] = 0; } //everyone in the spanningSet start unvisited

    int stamp = 1;
    for(auto node : spanningSet){
        if(visited[node] != 0) continue;

        //For an unvisited node, linearly search its connected component
        dfs(G,node,visited,stamp);

        std::vector<NodeId> conn_comp_vertex_set;
        std::map<NodeId,NodeId> conn_comp_id;
        //Get the reachable nodes
        for(NodeId v = 0; v < n ; v++){
            if(visited[v] == stamp){
                //The vector and map are used to quickly get the id in the new or old graph
                conn_comp_id[v] = conn_comp_vertex_set.size(); 
                conn_comp_vertex_set.push_back(v);
                
            }
        }

        //Build new 'Graph' object (easier to call blossom5 - no worries about the NodeIds)
        auto conn_comp_size = conn_comp_vertex_set.size();
        Graph component(conn_comp_size);
        for(NodeId u = 0; u < conn_comp_size; u++){
            auto node_in_G = G.node(conn_comp_vertex_set[u]);
            for(auto half_edge_id : node_in_G.outgoing_halfedges()){
                auto neighbor = G.halfedge(half_edge_id).target();
                if(visited[neighbor] == visited[conn_comp_vertex_set[u]] &&
                 conn_comp_id[neighbor] > u){ // we test conn_comp_id[neighbor] > u to make sure we are not doubling the edges
                     component.add_edge(u,conn_comp_id[neighbor], G.halfedge_weight(half_edge_id) );
                 }
            }
        }

        components.push_back( std::make_pair(component,conn_comp_vertex_set) );
        stamp++;
    }
    return components;
}

