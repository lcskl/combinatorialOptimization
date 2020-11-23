#ifndef MATCH_ALGO_HPP
#define MATCH_ALGO_HPP

/**
   @file match_algo.hpp

   @brief This file provides auxiliary data and methods to the maximum cardinality matching algorithm
**/

#include "graph.hpp"
#include <vector>
#include <set>
#include <memory>

struct Edge{
    ED::NodeId _x,_y;
    Edge(){}
    Edge(const ED::NodeId x,const ED::NodeId y){
        _x = std::min(x,y);
        _y = std::max(x,y);
    }
    bool operator <(const Edge& a) const{
        return std::tie(_x,_y) < std::tie(a._x, a._y);
    };
};

class Matching{
    public:
    std::vector<bool> _is_covered;
    std::set<Edge> _edges;


    Matching(int num_nodes){
        _size = 0;
        _is_covered.resize(num_nodes,false);
    }

    void print();

    void add_edge(Edge e);
    void remove_edge(Edge e);

    unsigned int cardinality();

    private:
        unsigned int _size;
};

namespace AT{ //Alternating Tree
    enum Parity {even, odd};
    
    class Tree{
        public:
        struct Node {
            ED::NodeId _graph_node_id;
            std::vector<std::shared_ptr<Node>> _children;

            Parity _par;

            Node() {}

            Node(const std::shared_ptr<Node>& parent, ED::NodeId id){
                std::shared_ptr<Node> new_node(new Node);
                new_node->_graph_node_id = id;
                new_node->_par = static_cast<Parity>((parent->_par+1) % 2);
                parent->_children.push_back( std::move(new_node) );
            }
        };
        std::shared_ptr<Node> _root;
        std::vector<bool> _in_tree;


        Tree(ED::NodeId id, int num_nodes_in_graph){
            std::unique_ptr<Node> root(new Node);
            root->_par = even;
            root->_graph_node_id = id;

            _root = std::move(root);
            _in_tree.resize(num_nodes_in_graph,false);
            _in_tree[id] = true;
            
        }

        void augment(Matching* M, ED::NodeId x,ED::NodeId y);
    };

}

Matching bipartite_perfect_matching(ED::Graph g);


#endif /* MATCH_ALGO_HPP */