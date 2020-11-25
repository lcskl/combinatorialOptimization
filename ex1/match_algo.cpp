#include "match_algo.hpp"


#include "graph.hpp" 
#include <iostream>
#include <stack>

void Matching::add_edge(Edge e){
    _edges.insert(e);
    _is_covered[e._x] = e._y;
    _is_covered[e._y] = e._x;
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
    std::cout << "Number of edges: " << _size << "- " << _edges.size() << std::endl;
    for(auto e : _edges){
        std::cout << e._x << "--" << e._y << std::endl;
    }
    std::cout << std::endl;
}

void AT::Tree::print(){
    std::cerr << "\n ###################  Tree  #######################\n";
    std::stack<std::shared_ptr<AT::Tree::Node>> S;
    if(_root == nullptr) std::cerr << "Null root!\n";
    S.push(_root);
    while(!S.empty()){
        auto current = S.top(); S.pop();
        if(current == nullptr) std::cerr << "Null pointer\n";
        std::cerr << "Current: " << current->_graph_node_id <<  " ";
        for(auto&& i : current->_children){
            std::cerr << "Ch: " << i->_graph_node_id <<  " ";
            S.push(i);
            std::cerr << "\n";
        }
    }
    std::cerr << "\n ##################################################\n";
}

std::shared_ptr<AT::Tree::Node> AT::Tree::find_node(ED::NodeId x){
    std::stack<std::shared_ptr<AT::Tree::Node>> S;
    S.push(_root);
    while(!S.empty()){
        auto current = S.top(); S.pop();

        if(current->_graph_node_id == x){
            return current;
        }

        for(auto&& i : current->_children)
            S.push(i);
    }
    return nullptr;
}

std::shared_ptr<AT::Tree::Node> AT::Tree::add_leaf(std::shared_ptr<Node> parent, ED::NodeId id){
    if(_root == nullptr) std::cerr << "Null root!\n";
    std::shared_ptr<Node> new_node = std::make_shared<Node>();
    new_node->_graph_node_id = id;
    new_node->_par = static_cast<Parity>((parent->_par+1) % 2);
    std::cerr << "adding as leaf\n";
    parent->_children.push_back( new_node );
    return new_node;
}

void AT::Tree::augment(Matching* M, ED::NodeId x,ED::NodeId y){
    std::cerr << "Augmenting\n";

    std::stack<std::shared_ptr<AT::Tree::Node>> S;
    std::vector<ED::NodeId> parent(_in_tree.size());

    parent[_root->_graph_node_id] = _root->_graph_node_id;

    S.push(_root);
    while(!S.empty()){
        auto current = S.top(); S.pop();

        std::cerr << current->_graph_node_id << std::endl;

        if(current->_graph_node_id == x){
            break;
        }

        for(auto&& i : current->_children){
            S.push(i);
            parent[i->_graph_node_id] = current->_graph_node_id;
        }
    }
    
    M->add_edge( Edge(x,y) );
    bool even = true;
    while(parent[x] != x){
        if(even){
            std::cerr << "Removing: " << parent[x] << "---" << x << std::endl;
            M->remove_edge( Edge(parent[x],x) );
        }else{
            
            std::cerr << "Adding: " << parent[x] << "---" << x << std::endl;
            M->add_edge( Edge(parent[x],x) );
        }
        x = parent[x];
        even = !even;
        M->print();
    }
    
}

void AT::Tree::extend(ED::NodeId x, ED::NodeId y, std::vector<Edge>& e, const ED::Graph& g, const Matching& M){
    std::cerr << "Extend tree through: " << x << "--" <<  y <<std::endl;
    auto x_in_tree = find_node(x);
    auto y_in_tree = add_leaf(x_in_tree,y);

    std::stack<std::shared_ptr<AT::Tree::Node>> S;
    S.push(y_in_tree);
    while(!S.empty()){
        auto current = S.top(); S.pop();
        if(current->_par == odd && M._is_covered[current->_graph_node_id] != -1 && find_node(M._is_covered[current->_graph_node_id])==nullptr){
            S.push(add_leaf(current,M._is_covered[current->_graph_node_id]));
        }else{
            for(auto neigh_id : g.node(current->_graph_node_id).neighbors()){
                if(find_node(neigh_id)==nullptr){
                    if(M._is_covered[neigh_id] != -1)
                        S.push(add_leaf(current,neigh_id));
                    else 
                        e.push_back( Edge(current->_graph_node_id,neigh_id) );
                }
            }
        }
    }

    print();
}



Matching bipartite_perfect_matching(ED::Graph g){
    std::cerr << "Starting on graph with n= " << g.num_nodes() <<  std::endl;

    Matching M(g.num_nodes());
    ED::NodeId first = 0;
    AT::Tree* T = new AT::Tree(first,g.num_nodes()); //Start with first vertex in node list
    std::vector<Edge> aug_edges;

    for(auto neigh : g.node(first).neighbors()){
        aug_edges.push_back( Edge(first,neigh) );
    }

    while( !aug_edges.empty() ){ 
        Edge current_edge = aug_edges.back(); aug_edges.pop_back();

        auto X = current_edge._x;
        auto Y = current_edge._y;

        std::cerr << "Current: " << current_edge._x << "--" <<  current_edge._y << std::endl;
        
        if( M._is_covered[Y] == -1){ // y is M-Exposed
            T->augment(&M , X, Y);
            if(2*M.cardinality() == g.num_nodes()){
                return M;
            }else{
                delete T;
                for(ED::NodeId i=0;i<g.num_nodes();i++){
                    if(M._is_covered[i] == -1){
                        first = i;
                        break;
                    }
                } 
                std::cerr << "New Tree Starting at: " << first << std::endl;
                T = new AT::Tree(first,g.num_nodes());
                T->print(); 
                aug_edges.clear();
                for(auto neigh : g.node(first).neighbors()){
                    aug_edges.push_back( Edge(first,neigh) );
                }
            }
        }else{
            T->extend(X,Y,aug_edges,g,M);
        }
    }
    return M;
}



