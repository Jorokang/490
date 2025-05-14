#include <iostream>
#include <string>
#include "gridmap.hpp"
#include "Astar.hpp"
#include "load_scens.hpp"
using namespace std;

void run(movingai::gridmap& g, movingai::scenario_manager& scenmrg) {

	Astar solver(g, g.width_, g.height_);
	for (int i=0; i<scenmrg.num_experiments(); i++) {
		auto expr = scenmrg.get_experiment(i);
		auto sx = expr->startx();
		auto sy = expr->starty();
		auto gx = expr->goalx();
		auto gy = expr->goaly();

		vector<int> parent;
		parent.assign(static_cast<size_t>(g.width_) * g.height_, -1);
		auto dist = solver.run(sx, sy, gx, gy, parent);
		printf("From (%d, %d) to (%d, %d) shortest distance: %.5f\n", sx, sy, gx, gy, dist);

		// TODO: postprocess
		// construct the path based on vector<int>parent;
		if (dist == -1) {
			printf("No path found\n");
		}
		else {
			vector<State> path;
			State current(gx, gy);
			while (current.x != sx || current.y != sy) {
				path.push_back(current);
				int parent_id = parent[current.y * g.width_ + current.x];
				current.x = parent_id % g.width_;
				current.y = parent_id / g.width_;
			}
			if (path.empty()) {
				printf("Path: (%d,%d)\n", sx, sy);
			}
			else {
				printf("Path: ");
				reverse(path.begin(), path.end());
				for (size_t j = 0; j < path.size(); ++j) {
                    printf("(%d,%d)%s", path[j].x, path[j].y, (j == path.size() - 1) ? "" : " -> ");
                }
                printf("\n");
			}
		}
	}
}

int main(int argc, char** argv) {
	// ./run_astar <mapfile> <scenfile>
	string mapfile = string(argv[1]);
	string scenfile = string(argv[2]);
	movingai::gridmap g(mapfile);
	movingai::scenario_manager scenmrg;
	scenmrg.load_scenario(scenfile);
	run(g, scenmrg);
}
