#pragma once
#include <algorithm>
#include <cassert>
#include <limits>
#include <math.h>
#include <queue>
#include <vector>
#include "gridmap.hpp"
using namespace std;
using namespace movingai;

class Astar {

  const double SQRT2 = 1.41421356237;

  struct Node {
    State loc;
    double g = 0; // what's the meaning of this variable?
    double h = 0; // waht's the meaning of this variable?

    Node(int x, int y, double g = 0.0, double h = 0.0) {
      this->loc = State{x, y};
      this->g = g;
      this->h = h;
    }

    inline bool isAt(const State &other) const {
      return loc.x == other.x && loc.y == other.y;
    }

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

public:
  int width, height;
  vector<double> gtable;
  gridmap grid;

  Astar(const gridmap &grid, int w, int h) : grid(grid), width(w), height(h) {
    gtable.resize(w * h);
  };

  inline int id(const State &loc) const { return loc.y * width + loc.x; }

  inline double hVal(const State &a, const State &b) {
    // no heuristic
    // return 0;

		// Question:
		// 1. what is admissibility?
		//  * what if heuristic is not admissible?
		// 2. what is consistency?
		//	* what if heuristic is not consistent?

    // TODO:
    // * what if use this heuristic?
    // * what's the meaning of this heuristic?
    int diag = min(abs(a.x - b.x), abs(a.y - b.y));
    int card = abs(a.x - b.x) + abs(a.y - b.y) - 2*diag;
    return card + diag * SQRT2;
  }

  inline double run(int sx, int sy, int gx, int gy, vector<int> &parent) {
    priority_queue<Node, vector<Node>, less<Node>> q;
    Node goal(gx, gy);
    Node start(sx, sy);
    start.h = hVal(start.loc, start.loc);
    gtable.assign(static_cast<size_t>(width) * height, numeric_limits<double>::max());
    parent.assign(static_cast<size_t>(width) * height, -1);

    assert(gtable.size() >= width * height);
		// std::fill(gtable.begin(), gtable.end(), numeric_limits<double>::max());
    gtable[id(start.loc)] = 0;

		// parent.resize(width * height);
    assert(parent.size() >= width * height);
		// std::fill(parent.begin(), parent.end(), -1);
    parent[id(start.loc)] = -1;

		// before move on, any other preconditions are not satisfied?

    q.push(start);

    while (!q.empty()) {
      Node c = q.top();
      q.pop();

      // what if (c.g > gtable[id(c.loc)]) ?
      // Is it possible?

      if (c.isAt(goal.loc)) {
        // TODO: at goal location, what to do?
        double optimalDist = gtable[id(c.loc)];; // what's the `optimalDist` suppose to be?
        return optimalDist;       // what if not return? `continiue` instead?
      }

      vector<State> successors = grid.get_neighbours(c.loc);
      for (auto &suc : successors) {
        double w = (c.loc.x == suc.x || c.loc.y == suc.y) ? 1 : SQRT2;
        auto x = suc.x;
        auto y = suc.y;
        // what if gtable[..] == c.g + w ?
        if (gtable[id(suc)] > c.g + w) {
          gtable[id(suc)] = c.g + w;
          parent[id(suc)] = id(c.loc);
          Node nxt = {x, y, c.g + w};
          nxt.h = hVal(nxt.loc, goal.loc);
          q.push(nxt);
        }
      }
    }
    return -1;
  }
};
