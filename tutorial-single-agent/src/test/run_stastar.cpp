#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <format>
#include "STAstar.hpp"
#include "dynscens.hpp"
#include "gridmap.hpp"
using namespace std;
namespace fs = std::filesystem;

void save_path(const vector<STAstar::STState>& path, string fn) {
	ofstream fout(fn);
	for (const auto& v: path) {
		fout << v.x << " " << v.y << " " << v.t << endl;
	}
}

void run(movingai::gridmap& g, dynenv::DynScen& scen, const string& output_dir_prefix) {

	STAstar solver(g, scen.node_constraints, g.width_, g.height_);
	auto sy = scen.source / g.width_;
    auto sx = scen.source % g.width_;
	for (auto t: scen.targetSet) {
		auto ty = t / g.width_;
        auto tx = t % g.width_;
		auto cost = solver.run(sx, sy, tx, ty);
		cout << format("[{}]({}, {}) to [{}]({}, {}): cost {}", 
				scen.source, sx, sy, t, tx, ty, cost) << endl;
		auto path = solver.get_path();
		assert (solver.validate(path));
		string plan_filename_base = to_string(scen.source) + "-" + to_string(t) + "-plan.txt";
            fs::path full_output_path = fs::path(output_dir_prefix) / plan_filename_base;
            
            save_path(path, full_output_path.string());
	}
}

string get_map_type_prefix(const string& filename) {
    fs::path p(filename);
    string stem = p.stem().string();

    if (stem.rfind("empty", 0) == 0) return "empty";
    if (stem.rfind("maze", 0) == 0) return "maze";
    if (stem.rfind("random", 0) == 0) return "random";
    if (stem.rfind("warehouse", 0) == 0) return "warehouse";
    
    return "unknown";
}

int main(int argc, char** argv) {
	// ./run_astar <mapfile> <scenfile>
	string mapfile = string(argv[1]);
	string scenfile = string(argv[2]);
	movingai::gridmap g(mapfile);
	dynenv::DynScen scen;
	vector<dynenv::DynScen> scens;
	string map_type = get_map_type_prefix(scenfile);
	dynenv::load_and_parse_json(scenfile, scens);
	fs::path scen_dir = fs::path(scenfile).parent_path(); // Gets the directory part, e.g., "../scens"
	fs::path stastar_dir = scen_dir / "stastar-res";
    fs::path full_path = scen_dir / map_type;
	run(g, scens[0], full_path);
}
