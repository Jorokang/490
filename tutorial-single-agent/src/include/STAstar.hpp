#pragma once
#include "dynscens.hpp"
#include "gridmap.hpp"
#include <algorithm>
#include <cassert>
#include <format>
#include <math.h>
#include <queue>
#include <set>
#include <tuple>
#include <vector>
using namespace std;

class STAstar {
public:
  using gridmap = movingai::gridmap;
  using Time = dynenv::Time;
  using vid = movingai::vid;
  using Cost = int;
  using ID = int;

  struct STState {
    vid x, y;
    Time t;
  };

  struct Node {
    STState v;
    Cost g = 0;
    Cost h = 0;

    Node(int x, int y, Time t = 0, Cost g = 0, Cost h = 0) {
      this->v = STState{x, y, t};
      this->g = g;
      this->h = h;
    }

    inline bool isAt(vid x, vid y) const { return v.x == x && v.y == y; }

    inline double f() const { return g + h; }

    // Define priority: which one should be expand earlier than others
    // what if modify the priority?
    inline bool operator<(const Node &rhs) const {
      if (f() == rhs.f())
        return g > rhs.g;
      else
        return f() > rhs.f();
    }
  };

  inline ID gen_node(int x, int y, Time t = 0, Cost g = 0, Cost h = 0) {
    if (nodes.size() + 1 >= nodes.capacity()) {
      nodes.reserve(nodes.capacity() * 2);
    }
    nodes.emplace_back(x, y, t, g, h);
    parent.push_back(-1);
    return nodes.size() - 1;
  }
  vector<Node> nodes;
  vector<int> parent;
  set<tuple<vid, vid, Time>> frontier;
  ID bestID, curID;
  Cost best;

  int width, height;
  const gridmap &grid;
  const dynenv::NodeCSTRs &cstrs;

  STAstar(const gridmap &g, const dynenv::NodeCSTRs &cs, int w, int h)
      : grid(g), cstrs(cs), width(w), height(h){};

  inline vid id(const vid &x, const vid &y) const { return y * width + x; }

  inline double hVal(const STState &a, const vid &gx, const vid &gy) {
		// using Manhattans distance in 4-connected grid
    return abs(a.x - gx) + abs(a.y - gy);
  }

  inline void init_search() {
		// TODO: init all data fields
    nodes.clear();
    parent.clear();
    frontier.clear();
    bestID = -1;
    curID = -1;
    best = -1;
  }

  inline bool is_safe(const vid &x, const vid &y, const vid &t) {
		// TODO: check whether (x, y, t) violate node constraints (cstrs) 
    if (x < 0 || x >= width || y < 0 || y >= height || t < 0)
      return false;
    if(grid.is_obstacle({x, y})) 
      return false; 
    long node_id = y * width + x;
    auto it = cstrs.find(node_id);
    if(it == cstrs.end()) 
      return true;
    else {
      for(auto intervals : it->second) {
        if(intervals.is_in(t))
          return false;
      }
      return true;
    }
    return false;
  }

  bool frontierCheck(vid x, vid y, Time t) {
    return frontier.find({x, y, t}) != frontier.end();
  }

	// Since `nodes` is dynamic container that get freqently resized
	// we must get the reference of a data entry by index
  inline const Node &cur() const { return this->nodes.at(curID); }

  Time get_target_critical_time(vid target_gx, vid target_gy) const {
      Time max_tr_at_target = -1;
      long target_node_id = id(target_gx, target_gy);

      auto it = cstrs.find(target_node_id);
      if (it != cstrs.end()) {
          if (it->second.empty()) {
              return 0;
          }
          for (const auto& constraint : it->second) {
              if (constraint.tr > max_tr_at_target) {
                  max_tr_at_target = constraint.tr;
              }
          }
      } else {
          return 0;
      }

      return (max_tr_at_target == -1) ? 0 : max_tr_at_target;
  }

  inline Cost run(int sx, int sy, int gx, int gy) {

    init_search();
    Time critical_time = get_target_critical_time(gx, gy);

		// The priority_queue only store index of the data,
		// So we need a customized comparetor
    auto pcmp = [&](const ID &i, const ID &j) {
      return this->nodes[i] < this->nodes[j];
    };
    priority_queue<int, vector<int>, decltype(pcmp)> q(pcmp);
    q.push(gen_node(sx, sy));

		// record the best objective and the corresponding node id
    best = bestID = -1;
    while (!q.empty()) {
      curID = q.top();
      q.pop();
      if(!is_safe(cur().v.x, cur().v.y, cur().v.t)) {
        continue;
      }
      if (cur().isAt(gx, gy)) {
        best = cur().g;
        bestID = curID;
        if (cur().g > critical_time) {
            break;
        } else {
            continue;
        }
      }

      // set the correct values  to model a 4-connected grid map:
      // four motions: up, down, left, right
      // each motion takes 1 time step
      const static int nummoves = 5;
      const static vid dx[] = {1, -1, 0, 0, 0};
      const static vid dy[] = {0, 0, 1, -1, 0};
      const static Cost w[] = {1, 1, 1, 1, 1};
      int num = 0;
      for (int i = 0; i < nummoves; i++) {
        num = i;
        vid nx = cur().v.x + dx[i];
        vid ny = cur().v.y + dy[i];
        Time nt = cur().v.t + w[i];
        if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
          continue;
        }

        if (grid.is_obstacle({nx, ny}) || !is_safe(nx, ny, nt)) {
          continue;
        }
				// Do we need this?
				// What's the purpose of this pruning?
        if (frontierCheck(nx, ny, nt)) {
           continue;
        }
        ID nid = gen_node(nx, ny, nt);
				// set g, h, parent value for the new node 
        nodes[nid].g = cur().g + w[i];
        nodes[nid].h = hVal(nodes[nid].v, gx, gy);
        parent[nid] = curID;
        frontier.insert({nx, ny, nt});
        q.push(nid);
      }
    }
    return best;
  }

  inline vector<STState> get_path() {
    vector<STState> res;
    ID cid = bestID;
    // TODO: extract the path
    if(cid == -1)
      printf("No path found\n");
    else {
      while(cid != -1) {
        res.push_back(nodes[cid].v);
        cid = parent[cid];
      }
    }
    reverse(res.begin(), res.end());
    return res;
  }

  inline bool validate(const vector<STState> &path) {
    // TODO: ensure a given path is validate
    for (const auto state : path) {
      if (!is_safe(state.x, state.y, state.t)) {
        return false;
      }
    }
    return true;
  }
};
