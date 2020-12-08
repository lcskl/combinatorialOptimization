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
#include <stack>
#include <algorithm>

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
    bool operator ==(const Edge& a) const{
        return (this->_x == a._x && this->_y == a._y) || (this->_y == a._x && this->_x == a._y);
    };
};

/*
    The Matching class is a collection of edges and a map of graph id's (vertices) to its matched counterpart (-1 by default)
*/
class Matching{
    public:
    std::vector<int> _is_covered; //Contains the matching pair (-1 if not convered by this matching)
    std::set<Edge> _edges;
    bool _perfect = false;

    Matching(int num_nodes){
        _size = 0;
        _is_covered.resize(num_nodes,-1);
    }

    void print(int n);

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
            std::shared_ptr<Node> _parent;

            Parity _par;
        };
        std::shared_ptr<Node> _root;
        std::vector<bool> _in_tree;
        std::vector<std::shared_ptr<Node>>_node_table; //mapping between node id's and its node structure in the tree

        //-------------------------  This is the abstraction from G' -> Graph after shrinkings ---------------------------------
        struct Cycle {
            ED::NodeId cycle_id;
            std::vector<ED::NodeId> shrunken_nodes;
            std::vector<std::pair<Edge,Edge>> redirect_edges;
        };

        std::vector<ED::NodeId> _label; //Label history for each vertex
        std::vector< std::vector<ED::NodeId> > _edges; //This will be updated after shrinkings! The idea is to leave the original graph intact.
        std::vector<Cycle> _shrinkings;
        //----------------------------------------------------------------------------------------------------------------------

        Tree(ED::NodeId id, const ED::Graph& g){
            std::shared_ptr<Node> root = std::make_shared<Node>();
            root->_par = even;
            root->_graph_node_id = id;
            root->_parent = nullptr; //Convention (It could be itself) - In traversals, remember to check parent!

            unsigned int num_nodes_in_graph = g.num_nodes();

            _root = root;
            _in_tree.resize(num_nodes_in_graph,false);
            _in_tree[id] = true;

            _edges.resize(num_nodes_in_graph);
            _node_table.resize(num_nodes_in_graph);
            _node_table[id] = _root;
            _label.resize(num_nodes_in_graph);

            for(ED::NodeId x = 0; x < num_nodes_in_graph ; x++){
                _label[x] = x; //Every vertex is labeled as its original id
                _edges[x] = g.node(x).neighbors();
            }
        }

        ~Tree(){
            std::stack<std::shared_ptr<Node>> S;
            S.push(_root);
            while(!S.empty()){
                auto current = S.top(); S.pop();
                current->_parent = nullptr;
                for(auto ch : current->_children)
                    S.push(ch);
            }
            _root = nullptr;
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
         * @param M Matching
         */
        void extend(ED::NodeId x, ED::NodeId y, std::vector<Edge>& e, const Matching& M);

        /**
         * Find node in try by its id (label)

         * @param x Label (NodeID) in orginal graph.
         * @return Reference to the node (nullptr if not exists)
         */
        std::shared_ptr<Node> find_node(ED::NodeId x);

        /**
         * Shrink Odd cycle through vertices x and y

         * @param x Label (NodeID) in {x,y}
         * @param y Label (NodeID) in {x,y}
         */
        void shrink(ED::NodeId x,ED::NodeId y, std::vector<Edge>& e, Matching &M);

        /**
         * Updates the current tree to be consistent with cycle shrinkings
         * Redirects outgoing children to the cycle representative and removes all vertices
         * of the cycle but the representative off the tree.
         * @param cycle_label Label (NodeID) of representative of a cycle
         */
        void update_tree(const std::vector<std::shared_ptr<Node>>& cycle, ED::NodeId cycle_label);

        /**
         * Undo the cycle shrinkings
         *
         * @param M Matching of the end graph G'(shrinked)
         * @return Matching M' obtained by unshrinking M
         */
        Matching unshrink(Matching M);

        unsigned int active_nodes();

        void print();

        void print_G_prime();
    };

}

Matching perfect_matching(ED::Graph g);
Matching max_matching(ED::Graph g, const std::string & filename);


#endif /* MATCH_ALGO_HPP */
