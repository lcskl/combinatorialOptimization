#ifndef CHECKER_HPP
#define CHECKER_HPP

#include "graph.hpp"
#include <deque>

std::pair <std::deque<MMC::NodeId>,double> min_mean_cycle(const MMC::Graph & G);

#endif