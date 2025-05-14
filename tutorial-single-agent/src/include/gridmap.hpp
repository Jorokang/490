#pragma once

#include <string>
#include <vector>

namespace movingai {

using namespace std;
using vid = int;

struct State {
  vid x, y;
};

class gridmap {
public:
  // init an empty map
  gridmap(int height, int width);
  // init map based on an input file
  gridmap(const string &filename);

  inline vector<State> get_neighbours(State c) const {
    // TODO: get neighbours of an 8-connected grid;
    auto res = vector<State>{};
    const int dx[] = {1, -1, 0,  0, 1, -1,  1, -1};
    const int dy[] = {0,  0, 1, -1, 1,  1, -1, -1};
    for (int i = 0; i < 8; ++i) {
      int next_x = c.x + dx[i];
      int next_y = c.y + dy[i];
      if (next_x >= 0 && next_x < width_ && next_y >= 0 && next_y < height_) {
        if (!is_obstacle({next_x, next_y})) {
          if (i < 4) {
            res.push_back({next_x, next_y});
          } 
          else {
            if (!is_obstacle({c.x + dx[i], c.y}) && !is_obstacle({c.x, c.y + dy[i]})) {
              res.push_back({next_x, next_y});
            }
          }
        }
      }
    }

    return res;
  };

  // get the label associated with the coordinate (x, y)
  inline bool is_obstacle(State c) const {
    auto label = this->get_label(c);
    return label == 1;
  }

  // set the label associated with the coordinate (x, y)
  inline void set_label(State c, bool label) { db[c.y * width_ + c.x] = label; }

  inline bool get_label(State c) const { return db[c.y * width_ + c.x]; }

  vid height_, width_;
  string filename;
  // whether a location is an obstacle
  vector<bool> db;
};

} // namespace movingai
