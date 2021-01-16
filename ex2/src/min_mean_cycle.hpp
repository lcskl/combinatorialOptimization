#ifndef MMC_HPP
#define MMC_HPP

/**
   @file min_mean_cycles.hpp

   @brief This file provides the actual min mean cycle algorithm and closely related methods
**/

#include <vector>
#include "graph.hpp"

std::vector<MMC::NodeId> minimum_mean_weight_cycle(MMC::Graph & g);

std::vector<MMC::NodeId> find_cycle(std::set<MMC::EdgeId> & J, MMC::Graph & g);

bool is_acyclic(const MMC::Graph & g);

std::vector<MMC::EdgeId> dfs_cycle(const MMC::Graph & g, MMC::NodeId root);

#endif