#include <vector>
#include <limits>
#include <algorithm> 
#include <iostream>
#include <fstream>
#include <string>
#include <filesystem>
#include <stdexcept>

#include "dynscens.hpp"   
#include "gridmap.hpp" 
#include "moving_target.hpp"
#include "mt_sipp.hpp"     
#include "hk_multi_mt_sipp.hpp"


int main(int argc, char* argv[]) {
    // Expected arguments: program_name map_file scen_file trackers_directory
    if (argc != 4) { 
        std::cerr << "Usage: " << argv[0] << " <mapfile> <scenfile_json> <trackers_directory>" << std::endl;
        std::cerr << "Example: " << argv[0] << " ../maps/maze-32-32-4.map ../scens/maze-100-10.json ../trackers/" << std::endl;
        return 1;
    }

    std::string map_file_path = argv[1];
    std::string json_scenario_path = argv[2];
    std::string trackers_directory_path = argv[3];

    movingai::gridmap g_map(map_file_path);
    int map_w = g_map.width_;
    int map_h = g_map.height_;

    std::vector<dynenv::DynScen> scenarios; 
    dynenv::load_and_parse_json(json_scenario_path, scenarios);
    
    const dynenv::DynScen& current_scenario = scenarios[0];
    const dynenv::NodeCSTRs& node_cstrs = current_scenario.node_constraints; 
    vid source_node_id = current_scenario.source; 

    std::vector<std::string> discovered_trajectory_files;
    try {
        std::filesystem::path trackers_dir(trackers_directory_path);
        if (!std::filesystem::exists(trackers_dir) || !std::filesystem::is_directory(trackers_dir)) {
            std::cerr << "Error: Trackers directory '" << trackers_directory_path << "' does not exist or is not a directory." << std::endl;
            return 1;
        }
        for (const auto& entry : std::filesystem::directory_iterator(trackers_dir)) {
            if (entry.is_regular_file()) {
                discovered_trajectory_files.push_back(entry.path().string());
            }
        }
        std::sort(discovered_trajectory_files.begin(), discovered_trajectory_files.end());
    } catch (const std::filesystem::filesystem_error& e) {
        std::cerr << "Error: Accessing trackers directory '" << trackers_directory_path << "' failed: " << e.what() << std::endl;
        return 1;
    }

    std::vector<STStateTracker> target_trackers(discovered_trajectory_files.size());
    for (size_t i = 0; i < discovered_trajectory_files.size(); ++i) {
        target_trackers[i].loadStatesFromFile(discovered_trajectory_files[i]);
    }

    vid agent_sx = source_node_id % map_w; 
    vid agent_sy = source_node_id / map_w; 
    Time agent_t0 = 0;

    movingai::State start_state_check = {agent_sx, agent_sy}; 

    MultiTargetInterceptor interceptor(g_map, node_cstrs, map_w, map_h, target_trackers);
    MultiTargetResult final_result = interceptor.run_multi_moving_sipp(agent_sx, agent_sy, agent_t0);

    if (final_result.success) {
        std::cout << "Succeed!" << std::endl;
        std::cout << "Total cost: " << final_result.total_time << std::endl;

        if (final_result.interception_order.size() == final_result.actual_interception_events.size()) {
            for (size_t i = 1; i < final_result.interception_order.size() + 1; ++i) {
                int target_true_idx = final_result.interception_order[i]; 
                const auto& intercept_event = final_result.actual_interception_events[i]; 
                std::cout << "  Intercepted Target " << target_true_idx 
                          << ": at (" << intercept_event.x 
                          << ", " << intercept_event.y 
                          << ") at time " << intercept_event.t << std::endl;
            }
        }

        std::cout << "Agent's full path (x y t):" << std::endl;
        for (const auto& state : final_result.full_path) {
            std::cout << "  " << state.x << " " << state.y << " " << state.t << std::endl;
        }
    }
    std::cout << "--- Test Finished ---" << std::endl;

    return 0;
}