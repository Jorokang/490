#pragma once
#include <algorithm>
#include <cassert>
#include <limits>
#include <math.h>
#include <queue>
#include <vector>
#include <set>
#include <tuple>
#include <map>
#include "gridmap.hpp"
#include "dynscens.hpp"
using namespace std;
using namespace movingai;


class SIPP {
public:
  using gridmap = movingai::gridmap;
  using Time = dynenv::Time;
  using vid = movingai::vid;
  using Cost = int;
  using ID = int;

    struct Time_interval {
        Time start;
        Time end;
    
        Time_interval(int start = 0, int end = 0) : start(start), end(end) {}

        bool operator<(const Time_interval& other) const {
            if (start != other.start) {
                return start < other.start;
            }
            return end < other.end;
        }

        bool operator==(const Time_interval& other) const {
            return start == other.start && end == other.end;
        }
    };

    struct SIPP_state {
        vid x;
        vid y;
        Time_interval interval;

        SIPP_state(vid x_coord, vid y_coord, Time_interval ti) : x(x_coord), y(y_coord), interval(ti) {}
        SIPP_state() : x(0), y(0), interval(0,0) {}

        bool operator<(const SIPP_state& other) const {
            if (x != other.x) return x < other.x;
            if (y != other.y) return y < other.y;
            return interval < other.interval;
        }

        bool operator==(const SIPP_state& other) const {
            return x == other.x && y == other.y && interval == other.interval;
        }
    };

    struct Node {
        SIPP_state state;
        Cost g;
        Cost h;
        Time arrival_time;

        Node(int x, int y, Time_interval interval, Cost g = 0, Cost h = 0, Time time = 0) {
            state = SIPP_state(x, y, interval);
            this->g = g;
            this->h = h;
            this->arrival_time = time;
        }

        inline bool isAt(vid x, vid y) const { return state.x == x && state.y == y; }

        inline double f() const { return g + h; }

        inline bool operator<(const Node &rhs) const {
            if (f() == rhs.f())
              return g > rhs.g;
            else
              return f() > rhs.f();
        }
    };

    vector<Node> nodes;
    vector<int> parent;
    set<tuple<vid, vid, Time_interval>> frontier;
    std::map<std::tuple<vid, vid, Time_interval>, Cost> state_g_values;
    ID bestID, curID;
    Cost best;

    int width, height;
    const gridmap &grid;
    const dynenv::NodeCSTRs &cstrs;

    std::map<vid, std::vector<Time_interval>> all_safe_intervals;
    Time max_time = std::numeric_limits<int>::max();

    inline ID gen_node(int x, int y, Time_interval interval = {0, 0}, Cost g = 0, Cost h = 0, Time arrival_t = 0) {
        if (nodes.size() + 1 >= nodes.capacity()) {
          nodes.reserve(nodes.capacity() * 2);
        }
        if (parent.size() >= parent.capacity()) {
            parent.reserve(std::max(static_cast<size_t>(1), parent.capacity() * 2));
        }
        nodes.emplace_back(x, y, interval, g, h, arrival_t);
        parent.push_back(-1);
        return nodes.size() - 1;
    }

    SIPP(const gridmap &g, const dynenv::NodeCSTRs &cs, int w, int h)
      : grid(g), cstrs(cs), width(w), height(h){};

    inline vid id(const vid &x, const vid &y) const { return y * width + x; }

    inline double hVal(const vid &x, const vid &y, const vid &gx, const vid &gy) {
		// using Manhattans distance in 4-connected grid
        return abs(x - gx) + abs(y - gy);
    }

    inline void init_search() {
        nodes.clear();
        parent.clear();
        frontier.clear();
        state_g_values.clear();
        bestID = -1;
        curID = -1;
        best = -1;

        init_all_safe_intervals();
    }

    void init_all_safe_intervals() {
        all_safe_intervals.clear();
        for (int x = 0; x < width; ++x) {
            for (int y = 0; y < height; ++y) {
                vid c_id = y * width + x;
                if (grid.is_obstacle({x, y})) {
                    all_safe_intervals[c_id] = {};
                    continue;
                }

                std::vector<Time_interval> unsafe_intervals;
                auto constraints = cstrs.find(c_id);
                if (constraints != cstrs.end()) {
                    for (const auto& constraint : constraints->second) {
                        unsafe_intervals.emplace_back(constraint.tl, constraint.tr);
                    }
                }

                std::sort(unsafe_intervals.begin(), unsafe_intervals.end());

                std::vector<Time_interval> safe_intervals;
                Time last_unsafe_end = -1;
                for (const auto& unsafe_interval : unsafe_intervals) {
                    if (unsafe_interval.start > last_unsafe_end + 1) {
                        safe_intervals.push_back({last_unsafe_end + 1, unsafe_interval.start - 1});
                    }
                    last_unsafe_end = std::max(last_unsafe_end, unsafe_interval.end);
                }

                if (last_unsafe_end < max_time -1) {
                    safe_intervals.push_back({last_unsafe_end + 1, max_time -1});
                }
                else if (unsafe_intervals.empty()) {
                    safe_intervals.push_back({0, max_time -1});
                }
                all_safe_intervals[c_id] = safe_intervals;
            }
        }
    }

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

    Cost run(vid sx, vid sy, vid gx, vid gy) {
        init_search();
        Time critical_time = get_target_critical_time(gx, gy);

		// The priority_queue only store index of the data,
		// So we need a customized comparetor
        auto pcmp = [&](const ID &i, const ID &j) {
            return this->nodes[i] < this->nodes[j];
        };
        priority_queue<int, vector<int>, decltype(pcmp)> q(pcmp);

        best = bestID = -1;
        vid grid_id = sy * width + sx;

        auto it_start_intervals = all_safe_intervals.find(grid_id);
        if (it_start_intervals == all_safe_intervals.end() || it_start_intervals->second.empty()) {
            return -1;
        }
        const std::vector<Time_interval>& safe_intervals = it_start_intervals->second;
        for (const auto& interval : safe_intervals) {
          q.push(gen_node(sx, sy, interval, interval.start, hVal(sx, sy, gx, gy), interval.start));
          frontier.insert({sx, sy, interval});
          state_g_values[{sx, sy, interval}] = interval.start;
        }

        while (!q.empty()) {
          curID = q.top();
          q.pop();
          if(!is_safe(cur().state.x, cur().state.y, cur().arrival_time)) {
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
          
            const static int nummoves = 5;
            const static vid dx[] = {1, -1, 0, 0, 0};
            const static vid dy[] = {0, 0, 1, -1, 0};
            const static Cost w[] = {1, 1, 1, 1, 1};

            for(int i = 0; i < nummoves; i++) {
                vid nx = cur().state.x + dx[i];
                vid ny = cur().state.y + dy[i];
                Time nt = cur().arrival_time + w[i];
                if(nx < 0 || nx >= width || ny < 0 || ny >= height || grid.is_obstacle({nx, ny})) {
                    continue;
                }
                if(all_safe_intervals.find(ny * width + nx) == all_safe_intervals.end())
                {
                    continue;
                }
                const std::vector<Time_interval>& safe_intervals = all_safe_intervals.find(ny * width + nx)->second;
                if(safe_intervals.empty()) {
                    continue;
                }
                for(const auto& interval : safe_intervals) {
                    Time new_arrival_time = std::max(nt, interval.start);
                    if(cur().state.interval.end < new_arrival_time - 1) {
                        continue;
                    }
                    if(new_arrival_time > interval.end || new_arrival_time < interval.start) {
                        continue;
                    }
                    if(frontier.count({nx, ny, interval}) && new_arrival_time >= state_g_values.at({nx, ny, interval})) {
                        continue;
                    }
                    ID nid = gen_node(nx, ny, interval, new_arrival_time, hVal(nx, ny, gx, gy), new_arrival_time);
                    q.push(nid);
                    parent[nid] = curID;
                    frontier.insert({nx, ny, interval});
                    state_g_values[{nx, ny, interval}] = new_arrival_time;
                }
            }


        }
        return best;
    }

    struct STState {
        vid x, y;
        Time t;
      };

    std::vector<STState> get_path() const {
        std::vector<STState> path;
        if (bestID == -1) {
            return path;
        }

        ID curID = bestID;
        while (curID != -1) {
            const Node& n = nodes.at(curID);
            path.emplace_back(n.state.x, n.state.y, n.arrival_time);
            curID = parent.at(curID);
        }
        std::reverse(path.begin(), path.end()); // Path is reconstructed backwards
        return path;
    }

    inline bool is_safe(const vid &x, const vid &y, const vid &t) const {
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

    inline bool validate(const vector<STState> &path) const {
        for (const auto state : path) {
          if (!is_safe(state.x, state.y, state.t)) {
            return false;
          }
        }
        return true;
    }


};


