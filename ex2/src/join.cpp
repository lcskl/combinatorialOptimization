#include "join.hpp"
#include "utils.hpp"
#include <iostream>
#include <map>
#include <cmath>

void minimum_weight_empty_join(const Graph & G){
    //This implementation is based on Theorems 51 and 52 of the lecture notes 

    auto T = G.odd_v_minus(); // Get V^-
    auto conn_comps = connected_components(G,T); //Components spanned by V^- with weight function d(e) := |c(e)| 

    std::cout << " Components of G(T):\n";
    for(auto comp : conn_comps){
        std::cout << comp.first << std::endl;
        for(NodeId x = 0; x < comp.second.size(); x++){
            std::cout << x << " -> " << comp.second[x] << std::endl; 
        }
    }
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
                     //REMEMBER: d(e) := |c(e)|
                     component.add_edge(u,conn_comp_id[neighbor], abs(G.halfedge_weight(half_edge_id)) );
                 }
            }
        }

        components.push_back( std::make_pair(component,conn_comp_vertex_set) );
        stamp++;
    }
    return components;
}

