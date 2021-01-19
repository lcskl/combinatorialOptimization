#include "utils.hpp"
#include <cmath>
#include <iostream>
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
        return (a.first > b.first);
    }
};

std::pair<std::vector<MMC::EdgeWeight>,std::vector<MMC::HalfEdgeId>> dijkstra(const MMC::Graph & G,MMC::NodeId start_node){
    auto infty = std::numeric_limits<EdgeWeight>::max();
    std::vector<EdgeWeight> distance(G.num_nodes());
    std::vector<HalfEdgeId> previous (G.num_nodes());
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
            EdgeWeight alt = distance[vertex] + G.halfedge_weight(out_hf);

            if(distance[vertex]!=infty && alt < distance[neighbor]){
                distance[neighbor] = alt;
                previous[neighbor] = G.halfedge(out_hf).inverse();
                queue.push(std::make_pair(alt,neighbor));
            }
        }
    }

    return std::make_pair(distance,previous);
}

std::pair<std::vector<MMC::EdgeWeight>,std::vector<MMC::HalfEdgeId>> floyd_warshall(const MMC::Graph & G){
    auto infty = std::numeric_limits<EdgeWeight>::max();
    std::vector<EdgeWeight> distances(G.num_nodes()*G.num_nodes(),infty);
    std::vector<HalfEdgeId> next(G.num_nodes()*G.num_nodes());

    for(NodeId vertex = 0; vertex < G.num_nodes(); vertex++){
        distances[vertex*G.num_nodes()+vertex] = 0;
        next[vertex*G.num_nodes()+vertex] = std::numeric_limits<HalfEdgeId>::max();

        for(auto out_hf : G.node(vertex).outgoing_halfedges()){
            auto neighbor = G.halfedge(out_hf).target();
            distances[vertex*G.num_nodes()+neighbor] = G.halfedge_weight(out_hf);
            next[vertex*G.num_nodes()+neighbor] = out_hf;
        }
    }

    for(NodeId k = 0; k < G.num_nodes(); k++){
        for(NodeId i = 0; i < G.num_nodes(); i++){
            for(NodeId j = 0; j < G.num_nodes(); j++){
                if(distances[i*G.num_nodes()+k] != infty &&  distances[k*G.num_nodes()+j] != infty){
                    if(distances[i*G.num_nodes()+j] > distances[i*G.num_nodes()+k] + distances[k*G.num_nodes()+j]){
                        distances[i*G.num_nodes()+j] = distances[i*G.num_nodes()+k] + distances[k*G.num_nodes()+j];
                        next[i*G.num_nodes()+j] = next[i*G.num_nodes()+k];
                    }
                }
            }
        }
    }

    // for(NodeId i = 0; i < G.num_nodes(); i++){
    //     for(NodeId j = 0; j < G.num_nodes(); j++){
    //         if(i!=j)
    //             std::cout << G.halfedge(next[i*G.num_nodes()+j]).target()+1 << " ";
    //         else
    //             std::cout << "I ";
    //     }
    //     std::cout << std::endl;
    // }

    return std::make_pair(distances,next);
}

Graph copy_abs_weight(const Graph & G){
    Graph G_d = G;

    for(EdgeId id = 0; id < G_d.num_edges(); id++){
        auto old_weight = G_d.edge_weight(id);
        G_d.set_edge_weight(id,fabs(old_weight));
    }

    return G_d;
}

void reduce_weights(Graph & g, double value)
{
    for(size_t i = 0; i < g.num_edges(); ++i)
    {
        g.set_edge_weight(i, g.edge_weight(i)-value);
    }
}