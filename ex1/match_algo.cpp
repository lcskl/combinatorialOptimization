#include "match_algo.hpp"


#include "graph.hpp" 
#include <iostream>
#include <stack>

void Matching::add_edge(Edge e){
    _edges.insert(e);
    _is_covered[e._x] = true;
    _is_covered[e._y] = true;
    _size++;
}

void Matching::remove_edge(Edge e){
    _size--;
    _edges.erase(e);
}

unsigned int Matching::cardinality(){
    return _size;
}

void Matching::print(){
    std::cout << "Number of edges: " << _size << std::endl;
    for(auto e : _edges){
        std::cout << e._x << "--" << e._y << std::endl;
    }
    std::cout << std::endl;
}

void AT::Tree::augment(Matching* M, ED::NodeId x,ED::NodeId y){
    std::cerr << "Augmenting\n";

    std::stack<std::shared_ptr<AT::Tree::Node>> S;
    std::vector<ED::NodeId> parent(_in_tree.size());

    parent[_root->_graph_node_id] = _root->_graph_node_id;

    S.push(std::move(_root));
    while(!S.empty()){
        auto current = S.top(); S.pop();

        if(current->_graph_node_id == x){
            break;
        }

        for(auto&& i : current->_children){
            S.push(std::move(i));
            parent[i->_graph_node_id] = current->_graph_node_id;
        }
    }
    
    M->add_edge( Edge(x,y) );
    bool even = true;
    while(parent[x] != x){
        if(even){
            M->remove_edge( Edge(parent[x],x) );
        }else{
            M->add_edge( Edge(parent[x],x) );
        }
        even = !even;
    }

}

Matching bipartite_perfect_matching(ED::Graph g){
    std::cerr << "Starting on graph with n= " << g.num_nodes() <<  std::endl;

    Matching M(g.num_nodes());
    ED::NodeId first = 0;
    AT::Tree T(first,g.num_nodes()); //Start with first vertex in node list
    std::vector<Edge> aug_edges;

    for(auto neigh : g.node(first).neighbors()){
        aug_edges.push_back( Edge(first,neigh) );
    }

    while( !aug_edges.empty() ){ 
        Edge current_edge = aug_edges.back(); aug_edges.pop_back();

        std::cerr << "Current: " << current_edge._x << "--" <<  current_edge._y << std::endl;
        
        if( !M._is_covered[current_edge._y] ){ // y is M-Exposed
            T.augment(&M,current_edge._x,current_edge._y);
            if(2*M.cardinality() == g.num_nodes()){
                return M;
            }else{
                

            }
        }


    }
    return M;
}



