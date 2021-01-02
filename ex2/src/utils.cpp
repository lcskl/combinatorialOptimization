#include "utils.hpp"

using namespace MMC;

void dfs(const Graph & G,NodeId vertex, std::vector<int> & visited,int stamp){
    visited[vertex] = stamp;

    auto node = G.node(vertex);
    for(auto half_edge : node.outgoing_halfedges()){
        auto neighbor = G.halfedge(half_edge).target();
        if(visited[neighbor] == 0)
            dfs(G,neighbor,visited,stamp);
    }
}