#include "match_algo.hpp"
#include "graph.hpp" 
#include "graph.hpp"
#include <iostream>
#include <stack>
#include <queue>
#include <algorithm>
#include <cstdio>


void Matching::add_edge(Edge e){
    _size += _edges.insert(e).second;
    _is_covered[e._x] = e._y;
    _is_covered[e._y] = e._x;
}

void Matching::remove_edge(Edge e){
    _size -= _edges.erase(e);
}

unsigned int Matching::cardinality(){
    return _size;
}


void Matching::print(int num_nodes_orig_graph){
    std::cout << "p " << num_nodes_orig_graph << " " << _size  << std::endl;
    for(auto e : _edges){
        std::cout << "e " << e._x+1 << " " << e._y+1 << std::endl;
    }
}

void AT::Tree::print(){
    std::cerr << "\n ###################  Tree  #######################\n";
    std::stack<std::shared_ptr<AT::Tree::Node>> S;
    if(_root == nullptr) std::cerr << "Null root!\n";
    S.push(_root);
    while(!S.empty()){
        auto current = S.top(); S.pop();
        if(current == nullptr) std::cerr << "Null pointer\n";
        std::cerr << "Current: " << current->_graph_node_id+1 << " P:" << ((current->_par == even)?'E':'O') <<  " Ch: ";
        for(auto&& i : current->_children){
            std::cerr << " " << i->_graph_node_id+1 <<  ", ";
            S.push(i);
        }
        std::cerr << "\n";
    }
    std::cerr << "\n ##################################################\n";
}

void AT::Tree::print_G_prime(){
   
    std::cerr << "-------------------- G PRIME -----------------------\n";

    for(auto const& x : _label){
        std::cerr << x.first+1 << " : ";
        for(auto l : x.second)
            std::cerr << l+1 << ", ";
        std::cerr << " <-\n"; 
    }
    std::cerr << "-------------------- LABEL / EDGES ----------------\n";
    for(ED::NodeId x = 0 ; x < _edges.size(); x++){
        if(_label[x].back() == x){
            std::cerr << x+1 << " : ";
            for(auto neigh : _edges[x]){
                std::cerr << neigh+1 << ", ";
            }
            std::cerr << "\n";
        }
    }
    std::cerr << "---------------------------------------------------\n";
}

std::shared_ptr<AT::Tree::Node> AT::Tree::find_node(ED::NodeId x){
    if(_in_tree[x]){
        return _node_table[x];
    }else{
        return nullptr;
    }
}

std::shared_ptr<AT::Tree::Node> AT::Tree::add_leaf(std::shared_ptr<Node> parent, ED::NodeId id){
    if(_root == nullptr) std::cerr << "Null root!\n";
    std::shared_ptr<Node> new_node = std::make_shared<Node>();
    new_node->_graph_node_id = id;
    new_node->_par = static_cast<Parity>((parent->_par+1) % 2);
    new_node->_parent = parent;

    parent->_children.push_back( new_node );

    _in_tree[id] = true;
    _node_table[id] = new_node;

    return new_node;
}
/*
    augment:
    Add the edge {x,y} to augment.
    Find a 'root <- x' path and alternatingly add and
    remove edges from the matching (alternating path)
*/
void AT::Tree::augment(Matching* M, ED::NodeId x,ED::NodeId y){
    M->add_edge( Edge(x,y) );
    bool even = true;
    auto x_in_tree = find_node(x);
    while(x_in_tree->_parent != nullptr){
        if(even){
            M->remove_edge( Edge(x_in_tree->_parent->_graph_node_id,x_in_tree->_graph_node_id) );
        }else{
            M->add_edge( Edge(x_in_tree->_parent->_graph_node_id,x_in_tree->_graph_node_id) );
        }
        x_in_tree = x_in_tree->_parent;
        even = !even;
    }

}
/*
    extend:
    Traverse the graph in a iterative DFS manner.
    Add necessary nodes to the tree and add edges of interest
    to the main edge list vector<Edge> e.
*/
void AT::Tree::extend(ED::NodeId x, ED::NodeId y, std::vector<Edge>& e, const Matching& M){
    auto x_in_tree = find_node(x);
    std::stack<std::pair<std::shared_ptr<Node>,ED::NodeId>> S;
    S.push(std::make_pair(x_in_tree,y));
    while(!S.empty()){
        auto top = S.top(); S.pop();

        if(find_node(top.second) != nullptr)
            continue;
        auto current = add_leaf(top.first,top.second);

        if(current->_par == odd && M._is_covered[current->_graph_node_id] != -1 && find_node(M._is_covered[current->_graph_node_id])==nullptr){
            S.push(std::make_pair(current,M._is_covered[current->_graph_node_id]));
        }else if(current->_par == even){
            for(auto neigh_id : _edges[current->_graph_node_id]){
                auto neigh_in_tree = find_node(neigh_id);
                if(neigh_in_tree==nullptr){
                    if(M._is_covered[neigh_id] != -1)
                        S.push(std::make_pair(current,neigh_id));
                    else 
                //printf("%lu, %u\n",neigh_id,n);
                auto neigh_in_tree = find_node(neigh_id);
                if(neigh_in_tree==nullptr){
                    //printf("%lu, %u\n",neigh_id,n);
                    if(M._is_covered[neigh_id] != -1){
                        S.push(std::make_pair(current,neigh_id));
                    }else
                        e.push_back( Edge(current->_graph_node_id,neigh_id) );
                }else{
                    if(neigh_in_tree->_par == even){
                        e.push_back( Edge(current->_graph_node_id,neigh_id) ); //Cycle of odd length found!
                    }
                }
            }
        }
    }
}

/*
    unshrink:
    Goes through all the odd shrunken odd cycles
    and add the respective edges to the matching  
*/
Matching AT::Tree::unshrink(Matching M){
    std::cerr << "Unshrinking\n";
    Matching M_prime = M;
    int aux = 0;
    while(!_shrinkings.empty()){
        std::cerr << "Step " << aux++ << std::endl;
    and add the respective edges to the matching
Matching AT::Tree::unshrink(Matching M){
    Matching M_prime = M;
    while(!_shrinkings.empty()){
        auto cycle = _shrinkings.back(); _shrinkings.pop_back();
        Edge x;
        ED::NodeId endpoint;
        for(auto edge : M_prime._edges){
            if(edge._x == cycle.cycle_id || edge._y == cycle.cycle_id){ //get the edge in the matching whose endpoint is a pseudonode
                x = edge;
                endpoint = (edge._x == cycle.cycle_id) ? edge._y : edge._x;
                break;
            }
        }

        ED::NodeId start = cycle.cycle_id;
        //If the current edge in the matching was in fact a redirection from a shrunken vertex, include this edge to the matching
        //(removing the one to the pseudo-node)
        for(auto e : cycle.redirect_edges){
            if(e.second == x){
                M_prime.remove_edge(x);
                M_prime.add_edge(e.first);
                start = (e.first._x == endpoint) ? (e.first._y) : (e.first._x);
            }
        }

        //One vertex of the cycle is matched. From it iterate circularly and add edges to the matching alternatingly
        auto init = std::find(cycle.shrunken_nodes.begin(),cycle.shrunken_nodes.end(),start) - cycle.shrunken_nodes.begin() + 1;
        for ( std::vector<ED::NodeId>::size_type i = 0; i < cycle.shrunken_nodes.size(); i+=2 )
        {
            if(cycle.shrunken_nodes[(init + i) % cycle.shrunken_nodes.size()]     == start ||
               cycle.shrunken_nodes[(init + i + 1) % cycle.shrunken_nodes.size()] == start)break;
            M_prime.add_edge( Edge(cycle.shrunken_nodes[(init + i) % cycle.shrunken_nodes.size()],
                                        cycle.shrunken_nodes[(init + i + 1) % cycle.shrunken_nodes.size()]));
        }
        //M_prime.print();
    }
    std::cerr << "Unshrinking done\n";
    }

    return M_prime;
}

/*
    shrink:
    Transforms the cycle in one edge (from G' perspective)
*/
void AT::Tree::shrink(ED::NodeId x,ED::NodeId y, std::vector<Edge>& e, Matching &M){
    Cycle new_cycle;
    auto x_in_tree = find_node(x);
    auto y_in_tree = find_node(y);
    //Find a x-root path
    std::vector<std::shared_ptr<Node>> x_root_path;
    auto current = x_in_tree;
    do{
        x_root_path.push_back(current);
        current = current->_parent;
    }while(current != nullptr);

    //Find a y-root path BUT stops when it finds an intersection with
    //the x-root path. After all, this will be the cycle representative (pseudonode)
    std::vector<std::shared_ptr<Node>> y_root_path;
    current = y_in_tree;
    do{
        if(std::find(x_root_path.begin(),x_root_path.end(),current) != x_root_path.end()){
            auto it = std::find(x_root_path.begin(),x_root_path.end(),current);
            x_root_path.resize(it-x_root_path.begin()+1);
            break;
        }else{
            y_root_path.push_back(current);
        }
        current = current->_parent;
    }while(current != nullptr);

    //Just getting the vertices of the cycle and the representative
    std::vector<std::shared_ptr<Node>> cycle;
    for(auto x : x_root_path) cycle.push_back(x);
    auto representative = cycle.back();
    for(auto y : y_root_path) cycle.push_back(y);
    auto cycle_label = representative->_graph_node_id;

    new_cycle.cycle_id = cycle_label;


    for(auto v_cycle : cycle){
        new_cycle.shrunken_nodes.push_back(v_cycle->_graph_node_id);

        if(_label[v_cycle->_graph_node_id] != cycle_label){
            _label[v_cycle->_graph_node_id] = cycle_label;
        }
    }

    //Redirecting the edges from the cycle to the outside
    for(auto v : cycle){
        if(v == representative){
            auto it = _edges[cycle_label].begin();
            while(it != _edges[cycle_label].end()){
                //std::cerr << "Removing edges from representative\n";
                auto neigh = *it;
                if(_label[neigh] == cycle_label){
                    M.remove_edge( Edge(cycle_label,neigh));
                    it = _edges[cycle_label].erase(it);
                }else
                    ++it;
            }
        }else{
            for(auto neighbor : _edges[v->_graph_node_id]){
                if(_label[neighbor] == neighbor && _label[neighbor] != cycle_label &&
                std::find(_edges[cycle_label].begin(),_edges[cycle_label].end(),neighbor)==_edges[cycle_label].end()){
                    _edges[cycle_label].push_back(neighbor);
                    std::replace(_edges[neighbor].begin(),_edges[neighbor].end(),v->_graph_node_id,cycle_label);

                    new_cycle.redirect_edges.push_back(std::make_pair(Edge(v->_graph_node_id,neighbor),Edge(cycle_label,neighbor)));

                    if(!_in_tree[neighbor] || find_node(neighbor)->_par == even){
                        e.push_back(Edge(cycle_label,neighbor));
                    }
                }else if(_label[neighbor] != neighbor && _label[neighbor] == cycle_label){
                    M.remove_edge( Edge(v->_graph_node_id,neighbor) );
                }
            }
        }
    }

    _shrinkings.push_back(new_cycle);

    update_tree(cycle,cycle_label);
}

void AT::Tree::update_tree(const std::vector<std::shared_ptr<Node>>& cycle, ED::NodeId cycle_label){
    auto rep = find_node(cycle_label);
    for(auto node : cycle){
        //For all cycle nodes apart from representative, redirect all outside children to representative
        if(node->_graph_node_id != cycle_label){
            for(auto child : node->_children){
                if(_label[child->_graph_node_id].back() != cycle_label){
                if(_label[child->_graph_node_id] != cycle_label){
                    child->_parent = rep;
                    child->_par = static_cast<Parity>((rep->_par+1) % 2);
                    rep->_children.push_back(child);
                }else{
                    child->_parent = nullptr; //Break circular dependency
                }
            }
        }
    }
    //For representative of the cycle - remove all children that are in cycle (this nodes do not exist from here on)
    auto it = rep->_children.begin();
    while( it != rep->_children.end()){
        auto child = *it;
        if(_label[child->_graph_node_id] == cycle_label){
            _in_tree[child->_graph_node_id] = false;
            _node_table[child->_graph_node_id] = nullptr;
            it = rep->_children.erase(it);
        }else{
            ++it;
        }
    }
}

unsigned int AT::Tree::active_nodes(){
    unsigned int count = 0;
    for(unsigned int i = 0; i < _in_tree.size() ; i++){
        if(_label[i] == i)
            count++;
    }
    return count;
}

Matching max_matching(ED::Graph g, const std::string & filename){
    size_t best = 0;
    int lowerBound = 0;
    int upperBound = g.num_nodes()-1;
    int r = 0;

    //if perfct matching exists
    if(g.num_nodes()%2==0) {
        //std::cout << "Try to find perfect matching." << std::endl;

        Matching M_perf = perfect_matching(g);

        if(M_perf._perfect == true)
            return M_perf;
        //else
            //std::cout << "No perfect matching found." << std::endl;
    }

    while (lowerBound < upperBound){ //<=?
        int k = lowerBound + ((upperBound - lowerBound) / 2); //mid
        if((k+(int)g.num_nodes()) % 2 == 1) k--; //otherwise there would not be a perf matching. use --, otherwise you would skip a case??

        //std::cout << lowerBound << " " << k << " " << upperBound << std::endl;
        //std::cout << "Binary search. Added " << k << " new nodes." << std::endl;

        //build graph with k completely added vertices, make a function for it?
        //start with g
        ED::Graph graph = ED::Graph::build_graph(filename);

        //add k nodes
        graph.add_nodes(k);

        //connect this nodes completely
        for(int i = 0; i < k; ++i){
            for(size_t j = 0; j < g.num_nodes(); ++j){
                //std::cout << "e " << g.num_nodes()+i << "," << j << std::endl;
                graph.add_edge(g.num_nodes()+i, j);
            }
        }

        Matching _M = perfect_matching(graph);

        if(_M._perfect == false) {  //not enough nodes
                lowerBound = k + 1;
        }
        else{
            //have to look at matching edges that do not cover the added nodes
            Matching M_new(graph.num_nodes());

           //only add edges not covering the added nodes
            for(auto e: _M._edges)
            {
                if(e._x < g.num_nodes() and e._y < g.num_nodes()) M_new.add_edge(e);
            }

            upperBound = k - 1;

            if(M_new.cardinality() > best){
                best = M_new.cardinality();
                r = k;  //best result with r new vertices
            }
        }

    }

    //find matching for best case (r added nodes), have to build it new (inefficient)
    //start with g
    ED::Graph graph = ED::Graph::build_graph(filename);

    //add r nodes
    graph.add_nodes(r);

    //connect this nodes completely
    for(int i = 0; i < r; ++i){
        for(size_t j = 0; j < g.num_nodes(); ++j){
            //std::cout << "e " << g.num_nodes()+i << "," << j << std::endl;
            graph.add_edge(g.num_nodes()+i, j);
        }
    }

    Matching M = perfect_matching(graph);

    //have to look at matching edges that do not cover the added nodes
    Matching M_perf(g.num_nodes());

   //only add edges not covering the added nodes
   for(auto e: M._edges)
   {
        if(e._x < g.num_nodes() and e._y < g.num_nodes()) M_perf.add_edge(e);
   }

    return M_perf;
}

Matching perfect_matching(ED::Graph g){
    Matching M(g.num_nodes());
    ED::NodeId first = 0;
    AT::Tree* T = new AT::Tree(first,g); //Start with first vertex in node list
    std::vector<Edge> aug_edges;

    for(auto neigh : g.node(first).neighbors()){
        aug_edges.push_back( Edge(first,neigh) );
    }

    while( !aug_edges.empty() ){
        Edge current_edge = aug_edges.back(); aug_edges.pop_back();

        auto X = current_edge._x;
        auto Y = current_edge._y;

        std::cerr << "Current Edge in Main Loop: " << current_edge._x+1 << "--" <<  current_edge._y+1 << std::endl;
        
        if(T->_label[X].back() != X || T->_label[Y].back() != Y){ 
            std::cerr << "One of the vertices shrunk!\n";
            continue;
        }

        if(T->_in_tree[Y]==false && M._is_covered[Y] == -1){ // y is not in Tree and is M-Exposed
            T->augment(&M , X, Y);
            auto M_prime = T->unshrink(M);
            if(2*M_prime.cardinality() == g.num_nodes()){
                M_prime._perfect = true;
        if(T->_label[X] != X || T->_label[Y] != Y){
            continue;
        }

        if(T->_in_tree[Y]==false && M._is_covered[Y] == -1){ // y is not in Tree and is M-Exposed´

            T->augment(&M , X, Y);
            auto M_prime = T->unshrink(M);

            if(2*M_prime.cardinality() == g.num_nodes()){
                M_prime._perfect = true;
                delete T;
                return M_prime;
            }else{
                delete T;
                M = M_prime;
                first = -1;

                for(ED::NodeId i=0;i<g.num_nodes();i++){
                    if(M._is_covered[i] == -1){
                        first = i;
                        break;
                    }
                }

                T = new AT::Tree(first,g);
                aug_edges.clear();
                for(auto neigh : T->_edges[first]){
                    aug_edges.push_back( Edge(first,neigh) );
                }

            }
        }else{
            if(T->_in_tree[Y] == false && M._is_covered[Y] != -1){
                T->extend(X,Y,aug_edges,M);
            }else if(T->find_node(Y)->_par == AT::even){
                T->shrink(X,Y,aug_edges,M);
            }
        }
    }
    M._perfect = false;
    return M;
}



