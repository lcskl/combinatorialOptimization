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
#include <iostream>

/*
    The Edge class is a simple wrapper for a pair of node id's.
    It maintains order of inclusion and overloads the less than comparison so that a set of edges does
    not take into account the order of those id's (undirected edge)
*/
struct Edge{
    ED::NodeId _x,_y;
    Edge(){}
    Edge(const ED::NodeId x,const ED::NodeId y){
        _x = x;
        _y = y;
    }
    bool operator <(const Edge& a) const{
        ED::NodeId min1_x = std::min(this->_x,this->_y); ED::NodeId min1_y = std::max(this->_x,this->_y);
        ED::NodeId min2_x = std::min(a._x,a._y); ED::NodeId min2_y = std::max(a._x,a._y);
        if(min1_x != min2_x)
            return (min1_x < min2_x);

        return (min1_y < min2_y);
    };
};

/*
    The Matching class is a collection of edges and a map of graph id's (vertices) to its matched counterpart (-1 by default)
*/
class Matching{
    public:
    std::vector<int> _is_covered; //Contains the matching pair (-1 if not convered by this matching)
    std::set<Edge> _edges;


    Matching(int num_nodes){
        _size = 0;
        _is_covered.resize(num_nodes,-1);
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

    //Class for an Alternating Tree
    class Tree{
        public:
        //This Node Class represents an abstraction of a node in the alternating tree perspective
        struct Node {
            ED::NodeId _graph_node_id; //NodeId (Label) in the underlying graph
            std::vector<std::shared_ptr<Node>> _children;

            Parity _par;

            Node() {}
            ~Node(){
                std::cerr << "Deleting " << _graph_node_id << std::endl;
            }
        };
        std::shared_ptr<Node> _root;
        std::vector<bool> _in_tree;


        Tree(ED::NodeId id, int num_nodes_in_graph){
            std::shared_ptr<Node> root = std::make_shared<Node>();
            root->_par = even;
            root->_graph_node_id = id;

            _root = root;
            _in_tree.resize(num_nodes_in_graph,false);
            _in_tree[id] = true;
            
        }
        /**
         * Add a node as children of some already inserted node.
         *
         * @param parent Reference to a node in the alternating tree.
         * @param id Label (NodeID) in the underlying graph.
         * @return Reference to the new created node
         */
        std::shared_ptr<Node> add_leaf(std::shared_ptr<Node> parent, ED::NodeId id);

        /**
         * Augment current Matching with edge {x,y} (x in Even(T)).
         *
         * @param M Reference to a to be augmented matching.
         * @param x Label (NodeID) in {x,y}.
         * @param y Label (NodeID) in {x,y}.
         */
        void augment(Matching* M, ED::NodeId x,ED::NodeId y);

        /**
         * Extend current alternating tree through edge {x,y} (x in Even(T)).
         *
         * @param x Label (NodeID) in {x,y}.
         * @param y Label (NodeID) in {x,y}.
         * @param e List of edges to be tested in the main loop of matching algorithm
         * @param g Underlying Graph - It needs the neighborhood not in the Tree
         * @param M Matching
         */
        void extend(ED::NodeId x, ED::NodeId y, std::vector<Edge>& e, const ED::Graph& g, const Matching& M);

        /**
         * Find node in try by its id (label)

         * @param x Label (NodeID) in orginal graph.
         * @return Reference to the node (nullptr if not exists)
         */
        std::shared_ptr<Node> find_node(ED::NodeId x);

        void print();
    };

}

Matching bipartite_perfect_matching(ED::Graph g);


#endif /* MATCH_ALGO_HPP */