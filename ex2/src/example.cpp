#include <iostream> // For writing to the standard output.
#include <fstream> // For reading input files.
#include <limits>

#include "graph.hpp"
#include "join.hpp"
#include "utils.hpp"
#include "../blossom5-v2.05.src/PerfectMatching.h"

void reduce_weights(Graph & g, double value)
{
    for(size_t i = 0; i < g.num_edges(); ++i)
    {
        g.set_edge_weight(i, g.edge_weight(i)-value);
    }
}

double abs2(double value)
{
    if (value >= 0) return value;
    else return -value;
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
        if(g.halfedge(edge_id).target() == peak)  //Fehler hier?? TAKE HALFEDGES FOR C, edge_id*2 could be wrong!
            in_cycle = true;                        // oder counter, wie oft ich bei peak war, dann if(g.halfedge(edge_id*2).target() == peak or g.halfedge(edge_id*2+1).target() == peak)
    }                                               // 3. Mal: 1. reingehen, 2. rausgehen aus peak, 3. wieder rein !!

    return final_cycle;
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


std::vector<NodeId> find_cycle(std::set<EdgeId> & J, Graph & g)
{
    Graph GJ = Graph(g.num_nodes());
    /*
    //print (empty set)-join J
    for(auto edge_id : J)
    {
        std::cout << "J: " << edge_id << " -> " << g.halfedge(2*edge_id).target()+1 << " -- " << g.halfedge(2*edge_id+1).target()+1 << std::endl;
    }*/

    NodeId v;   //TODO can J be empty here?

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

        if(weightJ == 0)
        {
            /*
             if(J[counter].size() != 0)
             {
                std::vector<NodeId> C = find_cycle(J[counter], g);
                return C;
             }
            else
            {
                //std::vector<NodeId> C = find_cycle(last, g);    //TODO can last be empty?
                std::vector<NodeId> C;
                if(counter == 0)
                    C = find_cycle(J[1], g);
                else
                    C = find_cycle(J[0], g);
                return C;
            }*/
            stop = true;
            break;  //dont want to change current
        }
        else
        {
            double gamma2 = weightJ / J[counter].size();
            std::cout << "gamma2 " << gamma2 << std::endl;
            if(abs2(gamma2) < epsilon) stop = true;

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

int main(int argc, char** argv)
{
   // If you don't want to write MMC:: all the time,
   // using a namespace in the implementation of some function is perfectly ok.
   using namespace MMC;

   if (argc != 3)
   {
      std::cout << "Expected exactly two arguments (path to input graph and path to output graph)!" << std::endl;
      return EXIT_FAILURE; // return 1 would do the same, but is way too easy to mix up!
   }
   std::fstream input_file{argv[1]};
   if (input_file.fail())
   {
      std::cout<<"Failed to open the input file. Exiting."<<std::endl;
      return EXIT_FAILURE;
   }
   std::ofstream output_file(argv[2], std::ios::out | std::ios::trunc);
   if (output_file.fail())
   {
      std::cout<<"Failed to open the output file. Exiting."<<std::endl;
      return EXIT_FAILURE;
   }
   Graph graph = Graph::read_dimacs(input_file);

   //minimum_weight_empty_join(graph);
   //std::set<EdgeId> a = minimum_weight_empty_join(graph);

/*
   std::vector<EdgeId> C = dfs_cycle(graph, 0);
   for(size_t i = 0; i < C.size(); ++i)
    {
        std::cout << "e " << graph.halfedge(2*C[i]).target()+1 << " " << graph.halfedge(2*C[i]+1).target()+1 << std::endl;
    }
*/

    std::vector<EdgeWeight> initialWeights(graph.num_edges());  //save them because they get modified
    for(size_t i = 0; i < graph.num_edges(); ++i ) initialWeights[i] = graph.edge_weight(i);    //TODO better, save in output file

    std::vector<NodeId> C = minimum_mean_weight_cycle(graph);

    std::cout << "p edge " << graph.num_nodes() << " " << C.size() << std::endl;
    for(size_t i = 0; i < C.size(); ++i)
    {
        std::cout << "e " << graph.halfedge(2*C[i]).target()+1 << " " << graph.halfedge(2*C[i]+1).target()+1 << " " << initialWeights[C[i]] << std::endl;
    }

}

