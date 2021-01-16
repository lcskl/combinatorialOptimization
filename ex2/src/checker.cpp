#include "checker.hpp"

#include <vector>
#include <stack>
#include <deque>
#include <limits>
#include <iostream>
using namespace MMC;

void all_cycles_search(const Graph & G, NodeId current, std::vector<bool> & in_rec_tree, double* min_value, 
                       std::deque<NodeId> & min_cycle, std::stack<std::pair<NodeId,double>> & last){
    in_rec_tree[current] = true;

    //std::cout << "In: " << current << " From: " << ( (last.empty()) ? -1 : last.top().first) << std::endl;

    for(auto out_hf_id : G.node(current).outgoing_halfedges()){
        auto neighbor = G.halfedge(out_hf_id).target();
        if(in_rec_tree[neighbor] == false){
            last.push(std::make_pair(current,G.halfedge_weight(out_hf_id)));
            all_cycles_search(G,neighbor,in_rec_tree,min_value,min_cycle,last);
            last.pop();
        }else if(!last.empty() and neighbor != last.top().first){
            //std::cout << "Cycle Found!\n";

            std::deque<NodeId> cycle;
            std::deque<double> edge_weights;
            double value = G.halfedge_weight(out_hf_id);
            uint32_t cycle_size = 1;

            cycle.push_back(current);
            //std::cout << current << " -> ";
        
            edge_weights.push_back(G.halfedge_weight(out_hf_id));

            do{
                cycle_size++;
                value += last.top().second;
                //std::cout << last.top().first << " >-> ";
                cycle.push_back(last.top().first);
                edge_weights.push_back(last.top().second);
                last.pop();                
            }while(cycle.back() != neighbor);
            //std::cout << std::endl;

            if(*min_value > (value/cycle_size)){
                *min_value = value/(double)cycle_size;
                min_cycle = cycle;
            }

            cycle.pop_front();
            while (!cycle.empty()) {
                //std::cout << cycle.front() << " >-> ";
                last.push(std::make_pair(cycle.back(),edge_weights.back()));
                cycle.pop_back();
                edge_weights.pop_back();
            }
            //std::cout << std::endl;
        }
    }
    in_rec_tree[current] = false;
}

std::pair <std::deque<NodeId>,double> min_mean_cycle(const Graph & G){
    double min_mean_weight = std::numeric_limits<double>::max();
    std::vector<bool> rec_tree(G.num_nodes(),false);
    std::deque<NodeId> min_cycle;
    std::stack<std::pair<NodeId,double>> last;

    for(NodeId v = 0; v < G.num_nodes(); v++){
        //std::cout << "START IN " << v << std::endl;
        all_cycles_search(G,v,rec_tree, &min_mean_weight, min_cycle, last);
    }

    return std::make_pair(min_cycle,min_mean_weight);
}

