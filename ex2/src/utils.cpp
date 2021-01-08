#include "utils.hpp"
#include <cmath>
#include <queue>
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

class Compare
{
public:
    bool operator() (std::pair<EdgeWeight,NodeId> a, std::pair<EdgeWeight,NodeId> b)
    {
        return (a.first < b.first);
    }
};

std::pair<std::vector<MMC::EdgeWeight>,std::vector<MMC::NodeId>> dijkstra(const MMC::Graph & G,MMC::NodeId start_node){
    auto infty = std::numeric_limits<EdgeWeight>::max();
    std::vector<EdgeWeight> distance(G.num_nodes());
    std::vector<NodeId> previous (G.num_nodes());
    std::vector<bool> processed (G.num_nodes(),false); 

    distance[start_node]=0;
    for(NodeId id = 0; id < G.num_nodes(); id++){
        if(id != start_node){
            distance[id] = infty;
            previous[id] = id; //As loops are out of question, previous[id]=id means UNDEFINED
        }
    }

    std::priority_queue<std::pair<EdgeWeight,NodeId>,std::vector<std::pair<EdgeWeight,NodeId>>,Compare> queue;
    queue.push(std::make_pair(0,start_node));

    while (!queue.empty()){
        auto next = queue.top();
        queue.pop();

        auto vertex = next.second;
        if(processed[vertex])continue; //This is a Lazy Deletion Implementation - Check Memory/Time afterwards
        processed[vertex] = true;

        for(auto out_hf : G.node(vertex).outgoing_halfedges()){
            auto neighbor = G.halfedge(out_hf).target();
            EdgeWeight alt = distance[vertex] + G.edge_weight(out_hf);

            if(alt < distance[neighbor]){
                distance[neighbor] = alt;
                previous[neighbor] = vertex;
                queue.push(std::make_pair(alt,neighbor));
            }
        }
    }

    return std::make_pair(distance,previous);
}

Graph copy_abs_weight(const Graph & G){
    Graph G_d(G.num_nodes());

    for(NodeId id = 0; id < G.num_nodes(); id++){
        auto vertex = G.node(id);
        for(auto outgoing_hf : vertex.outgoing_halfedges()){
            auto neighbor = G.halfedge(outgoing_hf).target();
            if(neighbor > id){
                G_d.add_edge(id,neighbor,abs(G.halfedge_weight(outgoing_hf)));
            }
        }
    }

    return G_d;
}