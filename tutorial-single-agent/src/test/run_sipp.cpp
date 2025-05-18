#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <format>
#include <filesystem>
#include "SIPP.hpp"
#include "dynscens.hpp"
#include "gridmap.hpp"

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
    auto sy = scen.source / g.width_;
    auto sx = scen.source % g.width_;

    fs::path output_dir_path(output_dir_prefix);

    for (auto t : scen.targetSet) {
        auto ty = t / g.width_;
        auto tx = t % g.width_;
        
        auto cost = solver.run(sx, sy, tx, ty);
        cout << format("[{}]({}, {}) to [{}]({}, {}): cost {}",
                       scen.source, sx, sy, t, tx, ty, cost)
             << endl;

        auto path = solver.get_path();

        string plan_filename_base = to_string(scen.source) + "-" + to_string(t) + "-plan.txt";
        fs::path full_output_path = fs::path(output_dir_prefix) / plan_filename_base;

        save_path(solver, path, full_output_path.string());
    }
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
    printf("Output dir: %s\n", full_output_dir_prefix.string().c_str());
    
    if (!scens.empty()) {
        run(g, scens[0], full_output_dir_prefix.string());
    }

    return 0;
}