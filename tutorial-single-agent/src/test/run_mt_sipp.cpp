#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <format>
#include <filesystem>
#include "SIPP.hpp"
#include "dynscens.hpp"
#include "gridmap.hpp"
#include "moving_target.hpp"
#include "mt_sipp.hpp"

namespace fs = std::filesystem;
using namespace std;

void save_path(const SIPP& solver, const std::vector<SIPP::STState>& path, const std::string& fn) {
    std::ofstream fout(fn);

    if (path.empty()) {
        return;
    }

    fout << path[0].x << " " << path[0].y << " " << path[0].t << std::endl;

    for (size_t i = 1; i < path.size(); ++i) {
        const auto& prev_v = path[i-1];
        const auto& current_v = path[i];

        for (SIPP::Time wait_t = prev_v.t + 1; wait_t < current_v.t; ++wait_t) {
            fout << prev_v.x << " " << prev_v.y << " " << wait_t << std::endl;
        }
        fout << current_v.x << " " << current_v.y << " " << current_v.t << std::endl;
    }
}

void run(movingai::gridmap& g, dynenv::DynScen& scen, const string& output_dir_prefix) {
    SIPP solver(g, scen.node_constraints, g.width_, g.height_);
    auto sy = 8;
    auto sx = 8;
    STStateTracker tracker;
    string filename = "../scens/sipp-res/maze/426-527-plan.txt";
    tracker.loadStatesFromFile(filename);
    auto tstart = std::chrono::steady_clock::now();
    for (Time t = 0; t < numeric_limits<Time>::max(); ++t) {
        auto current_target = tracker.getCoordinatesAtTime(t);
        Time cost = solver.run(sx, sy, current_target.first, current_target.second);
        if (cost !=-1 &&cost <= t) {
            for (int i = cost + 1; i < t; ++i) {
                if(!solver.is_safe(current_target.first, current_target.second, i))
                {
                    continue;
                }
                auto next_target = tracker.getCoordinatesAtTime(i);
                auto next_cost = solver.run(sx, sy, next_target.first, next_target.second);
                if(next_cost < cost)
                {
                    cost = next_cost;
                    current_target = next_target;
                    t = i;
                    break;
                }
            }
            printf("[%ld](%d, %d) to [%d](%d, %d): cost %d and time %d\n",
                       scen.source, sx, sy, current_target.first + g.width_* current_target.second, current_target.first, current_target.second, cost, t);
            break;
        }
        else {
            printf("fail ! [%ld](%d, %d) to [%d](%d, %d): cost %d and time %d\n",
                       scen.source, sx, sy, current_target.first + g.width_* current_target.second, current_target.first, current_target.second, cost, t);
        }
            
    }
    auto tnow = std::chrono::steady_clock::now();
    auto tcost = chrono::duration<double>(tnow - tstart).count();
    printf("SIPP:  runtime: %fs\n", tcost);

    mt_SIPP mt_solver(g, scen.node_constraints, filename,  g.width_, g.height_);
    tstart = std::chrono::steady_clock::now();
    auto mt_cost = mt_solver.run(sx, sy);
    tnow = std::chrono::steady_clock::now();
    tcost = chrono::duration<double>(tnow - tstart).count();
    auto found_target = tracker.getCoordinatesAtTime(mt_cost);
    printf("mt_SIPP:  runtime: %fs with cost %d at (%d, %d)\n", tcost, mt_cost, found_target.first, found_target.second);
    auto path = mt_solver.get_path();
    mt_solver.validate(path);
}

string get_map_type_prefix(const string& scen_filename) {
    fs::path p(scen_filename);
    string stem = p.stem().string(); 

    if (stem.rfind("empty", 0) == 0) return "empty";
    if (stem.rfind("maze", 0) == 0) return "maze";
    if (stem.rfind("random", 0) == 0) return "random";
    if (stem.rfind("warehouse", 0) == 0) return "warehouse";
    
    return "unknown";
}

int main(int argc, char** argv) {
    if (argc < 3) {
        cerr << "Usage: ./run_sipp <mapfile> <scenfile>" << endl;
        return 1;
    }

    string mapfile = string(argv[1]);
    string scenfile = string(argv[2]);

    movingai::gridmap g(mapfile);
    if (g.width_ == 0 || g.height_ == 0) {
        cerr << "Error loading map: " << mapfile << endl;
        return 1;
    }

    vector<dynenv::DynScen> scens;
    dynenv::load_and_parse_json(scenfile, scens); 

    if (scens.empty()) {
        cerr << "Error loading or parsing scenarios, or no scenarios found in: " << scenfile << endl;
        return 1;
    }

    string map_type = get_map_type_prefix(scenfile); 
    fs::path scen_dir = fs::path(scenfile).parent_path(); 
    fs::path output_base_dir = scen_dir / "sipp-res"; 
    fs::path full_output_dir_prefix = output_base_dir / map_type;
    
    if (!scens.empty()) {
        run(g, scens[0], full_output_dir_prefix.string());
    }

    return 0;
}