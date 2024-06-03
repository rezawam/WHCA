#include "include/Solver.hpp"

void WHCAPathFinder::FindPaths() {
    bool all_agents_found_path = false;
    while(!all_agents_found_path) {
        for (auto& agent : agents) {
            agent->FindPortionPath(space_time_map, graph);
        }
    }
}