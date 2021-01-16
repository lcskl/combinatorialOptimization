#include "min_mean_cycle.hpp"

#include "graph.hpp"
#include "join.hpp"
#include "utils.hpp"

#include <vector>
#include <iostream>
#include <cmath>

using namespace MMC;

std::vector<NodeId> minimum_mean_weight_cycle(Graph & g)
{
    double epsilon = 1.1e-16;
    bool stop = false;
    double gamma = -std::numeric_limits<double>::max();

    std::vector<std::set<EdgeId>> J(2);
    int counter = 0;

    if(is_acyclic(g) == true)
    {
        std::cout << "graph is acyclic" << std::endl;
        std::vector<NodeId> C;
        return C;
    }

    for(size_t i = 0; i < g.num_edges(); ++i)
    {
        if(gamma < g.edge_weight(i)) gamma = g.edge_weight(i);
    }

    reduce_weights(g, gamma);

    while(stop == false)
    {
        J[counter] = minimum_weight_empty_join(g);

        double weightJ = 0;
        for(auto edge : J[counter])
        {
            weightJ += g.edge_weight(edge);
        }

        std::cout << "weight " << weightJ << std::endl;

        if(weightJ == 0 || J[counter] == J[(counter+1)%2])
        {
            stop = true;
            break;  //dont want to change current
        }
        else
        {
            double gamma2 = weightJ / J[counter].size();
            std::cout << "gamma2 " << gamma2 << std::endl;
            if(fabs(gamma2) < epsilon) stop = true;

            reduce_weights(g, gamma2);
        }
        //so we can save the last one without copying the current
        if(counter == 0)
        {
            counter = 1;
        }
        else
        {
            counter = 0;
        }
    }

    if(J[counter].size() != 0)
    {
        std::vector<NodeId> C = find_cycle(J[counter], g);
        for(auto edge_id : J[counter])
        {
            std::cout << "J[counter]: " << edge_id << " -> " << g.halfedge(2*edge_id).target()+1 << " -- " << g.halfedge(2*edge_id+1).target()+1 << std::endl;
        }
        return C;
        }
    else
    {
        std::vector<NodeId> C;
        if(counter == 0)
        {
            C = find_cycle(J[1], g);
            for(auto edge_id : J[1])
            {
                std::cout << "J[1]: " << edge_id << " -> " << g.halfedge(2*edge_id).target()+1 << " -- " << g.halfedge(2*edge_id+1).target()+1 << std::endl;
            }
        }
        else
        {
            C = find_cycle(J[0], g);
            for(auto edge_id : J[0])
            {
                std::cout << "J[0]: " << edge_id << " -> " << g.halfedge(2*edge_id).target()+1 << " -- " << g.halfedge(2*edge_id+1).target()+1 << std::endl;
            }
        }
        return C;
    }
}

std::vector<NodeId> find_cycle(std::set<EdgeId> & J, Graph & g)
{
    Graph GJ = Graph(g.num_nodes());
    /*
    //print (empty set)-join J
    for(auto edge_id : J)
    {
        std::cout << "J: " << edge_id << " -> " << g.halfedge(2*edge_id).target()+1 << " -- " << g.halfedge(2*edge_id+1).target()+1 << std::endl;
    }*/

    NodeId v = 0;   //TODO can J be empty here?

    for(auto edge_id: J)
    {
        GJ.add_edge(g.halfedge(2*edge_id).target(), g.halfedge(2*edge_id+1).target(), edge_id);
        v = g.halfedge(2*edge_id).target();
    }

    /*
    //print graph GJ = (V(G), E(J))
    for(size_t edge_id = 0; edge_id < GJ.num_edges(); ++edge_id)
    {
        std::cout << "GJ: " << edge_id << " -> " << GJ.halfedge(2*edge_id).target()+1 << " -- " << GJ.halfedge(2*edge_id+1).target()+1 << " weight " << GJ.edge_weight(edge_id) << std::endl;

    }*/

    std::vector<EdgeId> C = dfs_cycle(GJ, v);

    return C;

}

bool is_acyclic(const Graph & g)
{
    std::vector<int> visited(g.num_nodes(), 0);
    std::vector<EdgeId> cycle;

    std::vector<NodeId> s;
    std::vector<NodeId> last(g.num_nodes());
    NodeId current;

    if(g.num_nodes() == 0)
        return true;

    for(size_t i = 0; i < visited.size(); ++i)  //have to check all components
    {
        if(visited[i] == 0)
        {
            last[i] = i;
            s.push_back(i);

            while(s.empty() == false)   //goes to the component containing i
            {
                current = s.back();
                s.pop_back();

                visited[current] = 1;

                for(auto half_edge : g.node(current).outgoing_halfedges())
                {
                    auto neighbor = g.halfedge(half_edge).target();
                    if(visited[neighbor] == false)
                    {
                        if(neighbor != last[current])
                        {
                            last[neighbor] = current;
                            s.push_back(neighbor);
                        }
                    }
                    else
                    {
                        if(neighbor != last[current])    //found cycle
                        {
                            return false;
                        }

                    }
                }
                //last = current;
            }
        }
    }

    return true;
}

std::vector<EdgeId> dfs_cycle(const Graph & g, NodeId root)
{
    std::vector<int> visited(g.num_nodes(), 0);
    std::vector<EdgeId> final_cycle;
    std::vector<HalfEdgeId> cycle;

    std::vector<NodeId> s;
    s.push_back(root);
    NodeId current, last, peak=0;
    bool in_cycle = false;
    last = root;
    int halfedge = -1;

    while(s.empty() == false)
    {
        current = s.back();
        s.pop_back();

        visited[current] = 1;
        if(current != root and halfedge != -1)
        {
            cycle.push_back(halfedge);   //dont do this for the starting node
            //std::cout << "adding " << g.edge_weight(edge) << std::endl;
        }

        for(auto half_edge : g.node(current).outgoing_halfedges())
        {
            auto neighbor = g.halfedge(half_edge).target();
            if(visited[neighbor] == false)
            {
                if(neighbor != last)
                {
                    //std::cout << "adding " << neighbor+1 << " to stack, current " << current+1 << std::endl;
                    s.push_back(neighbor);
                    halfedge = half_edge;
                }
            }
            else
            {
                if(neighbor != last)    //found cycle
                {
                    //std::cout << "adding " << g.edge_weight(half_edge/2) << std::endl;
                    cycle.push_back(half_edge);
                    //return cycle;
                    peak = neighbor;
                    s.clear();  //want to leave the while loop too
                    break;
                }
            }
        }
        last = current;
    }

    if(peak == root)
        in_cycle = true;

    for(size_t i = 0; i < cycle.size(); ++i)
    {
        EdgeId edge_id = cycle[i];
        if(in_cycle == true)
        {
            final_cycle.push_back(g.edge_weight(edge_id/2));    //here we saved the edge ids in the original graph
            //std::cout << "final adding " << edge_id << std::endl;
        }
        if(g.halfedge(edge_id).target() == peak)  
            in_cycle = true;                        
    }                                               

    return final_cycle;
}