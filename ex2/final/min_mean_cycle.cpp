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
    double epsilon = 1.1e-16;   //If gamma2 gets too small it does not make sense to continue
    bool stop = false;
    double gamma = -std::numeric_limits<double>::max();

    std::vector<std::set<EdgeId>> J(2); //save two joins so that we can easily get acces to the one of the previous iteration
    int counter = 0;

    J[1] = a_cycle(g);

    if(J[1].size() == 0)
    {
        std::cout << "graph is acyclic" << std::endl;
        std::vector<NodeId> cycle;
        return cycle;
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

        if(weightJ == 0 || J[counter] == J[(counter+1)%2])
        {
            stop = true;
            break;  //dont want to change current
        }
        else
        {
            double gamma2 = weightJ / J[counter].size();
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
        std::vector<NodeId> cycle = find_cycle(J[counter], g);
        /*for(auto edge_id : J[counter])
        {
            std::cout << "J[counter]: " << edge_id << " -> " << g.halfedge(2*edge_id).target()+1 << " -- " << g.halfedge(2*edge_id+1).target()+1 << std::endl;
        }*/
        return cycle;
        }
    else
    {
        std::vector<NodeId> cycle;
        if(counter == 0)
        {
            cycle = find_cycle(J[1], g);
            /*for(auto edge_id : J[1])
            {
                std::cout << "J[1]: " << edge_id << " -> " << g.halfedge(2*edge_id).target()+1 << " -- " << g.halfedge(2*edge_id+1).target()+1 << std::endl;
            }*/
        }
        else
        {
            cycle = find_cycle(J[0], g);
            /*for(auto edge_id : J[0])
            {
                std::cout << "J[0]: " << edge_id << " -> " << g.halfedge(2*edge_id).target()+1 << " -- " << g.halfedge(2*edge_id+1).target()+1 << std::endl;
            }*/
        }
        return cycle;
    }
}

std::vector<NodeId> find_cycle(std::set<EdgeId> & J, Graph & g)
{
    Graph GJ = Graph(g.num_nodes());

    NodeId v = 0;

    for(auto edge_id: J)    //GJ = graph (V(G), J)
    {
        GJ.add_edge(g.halfedge(2*edge_id).target(), g.halfedge(2*edge_id+1).target(), edge_id);
        v = g.halfedge(2*edge_id).target();
    }

    std::vector<EdgeId> cycle = dfs_cycle(GJ, v);

    return cycle;

}

std::set<EdgeId>  a_cycle(const Graph & g)
{
    std::vector<int> visited(g.num_nodes(), 0);
    std::set<EdgeId> cycle;
    std::vector<int> dfs_path;

    std::vector<NodeId> s;
    std::vector<NodeId> last(g.num_nodes());
    NodeId current,peak = 0;
    int lastHalfEdge = -1;
    bool in_cycle = false;
    bool cycle_found = false;

    if(g.num_nodes() == 0)
        return cycle;

    for(size_t i = 0; i < visited.size() && !cycle_found; ++i)  //have to check all components
    {
        if(visited[i] == 0)
        {
            last[i] = i;
            s.push_back(i);

            while(s.empty() == false)   //goes to the component containing i
            {

                current = s.back();
                s.pop_back();

                if(current != i and lastHalfEdge != -1)
                {
                    dfs_path.push_back(lastHalfEdge);   //dont do this for the starting node
                }


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
                            lastHalfEdge = half_edge;
                        }
                    }
                    else
                    {
                        if(neighbor != last[current])    //found cycle
                        {
                            dfs_path.push_back(half_edge);
                            peak = neighbor;
                            cycle_found = true;
                            s.clear();  //want to leave the while loop too

                            if(peak == i)
                                in_cycle = true;

                            break;
                        }
                    }
                }
            }
        }
    }

    for(size_t i = 0; i < dfs_path.size() && cycle_found; ++i)
    {
        HalfEdgeId edge_id = dfs_path[i];
        if(in_cycle == true)
        {
            cycle.insert(edge_id/2);    //here we saved the edge ids in the original graph
        }
        if(g.halfedge(edge_id).target() == peak)
            in_cycle = true;
    }

    return cycle;
}

/*Starting at the <root> we go through <g> with a DFS and save our path in <dfs_path>. When we visit a node twice,
we save this node <peak> and stop the DFS. Then we go through the path <dfs_path> again and when we visit <peak>
for the first time, we start saving the edges in <cycle> since here the found cycle begins.*/
std::vector<EdgeId> dfs_cycle(const Graph & g, NodeId root)
{
    std::vector<int> visited(g.num_nodes(), 0);
    std::vector<EdgeId> cycle;
    std::vector<HalfEdgeId> dfs_path;

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
            dfs_path.push_back(halfedge);   //dont do this for the starting node
        }

        for(auto half_edge : g.node(current).outgoing_halfedges())
        {
            auto neighbor = g.halfedge(half_edge).target();
            if(visited[neighbor] == false)
            {
                if(neighbor != last)
                {
                    s.push_back(neighbor);
                    halfedge = half_edge;
                }
            }
            else
            {
                if(neighbor != last)    //found cycle
                {
                    dfs_path.push_back(half_edge);
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

    for(size_t i = 0; i < dfs_path.size(); ++i)
    {
        EdgeId edge_id = dfs_path[i];
        if(in_cycle == true)
        {
            cycle.push_back(g.edge_weight(edge_id/2));    //here we saved the edge ids in the original graph
        }
        if(g.halfedge(edge_id).target() == peak)
            in_cycle = true;
    }

    return cycle;
}
