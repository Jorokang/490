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


class mt_SIPP {
public:
  using gridmap = movingai::gridmap;
  using Time = dynenv::Time;
  using vid = movingai::vid;
  using Cost = int;
  using ID = int;
  const Time INFT = numeric_limits<Time>::max() / 2;

    struct Time_interval {
        Time start;
        Time end;
        int key;
    
        Time_interval(int start = 0, int end = 0, int key = 0) : start(start), end(end), key(key) {}

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

    struct GVar {
        Cost g;
        int round;
    };

    struct mt_SIPP_state {
        vid x;
        vid y;
        Time_interval interval;

        mt_SIPP_state(vid x_coord, vid y_coord, Time_interval ti) : x(x_coord), y(y_coord), interval(ti) {}
        mt_SIPP_state() : x(0), y(0), interval(0,0) {}

        bool operator<(const mt_SIPP_state& other) const {
            if (x != other.x) return x < other.x;
            if (y != other.y) return y < other.y;
            return interval < other.interval;
        }

        bool operator==(const mt_SIPP_state& other) const {
            return x == other.x && y == other.y && interval == other.interval;
        }
    };

    struct Node {
        mt_SIPP_state state;
        Cost g;
        Cost h;
        Time arrival_time;

        Node(int x, int y, Time_interval interval, Cost g = 0, Cost h = 0, Time time = 0) {
            state = mt_SIPP_state(x, y, interval);
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
    //std::map<std::tuple<vid, vid, Time_interval>, Cost> state_g_values;
    vector<vector<GVar>> gtable;
    ID bestID, curID;
    Cost best;

    int width, height;
    int global_round = 0;
    const gridmap &grid;
    const dynenv::NodeCSTRs &cstrs;

    std::map<vid, std::vector<Time_interval>> all_safe_intervals;
    Time max_time = std::numeric_limits<int>::max() / 2;

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

    mt_SIPP(const gridmap &g, const dynenv::NodeCSTRs &cs, int w, int h)
      : grid(g), cstrs(cs), width(w), height(h){
        init_all_safe_intervals();
        gtable.resize(w * h);
        //tracker.loadStatesFromFile(filename);
        for (int i = 0; i < h * w; i++) {
            if (cstrs.find(i) == cstrs.end())
                gtable[i].resize(1);
            else
                gtable[i].resize(cstrs.at(i).size() + 1);
        }
      };

    inline vid id(const vid &x, const vid &y) const { return y * width + x; }

    inline double hVal(const vid &x, const vid &y, Time t, STStateTracker& tracker) {
        return tracker.getMinDistanceToPoint(x, y, t);
        
    }

    inline Cost gval(vid cid, int key) {
        if(gtable[cid][key].round == global_round) {
            return gtable[cid][key].g;
        }
        else {
            return INFT;
        }
    }

    inline void init_search() {
        nodes.clear();
        parent.clear();
        //state_g_values.clear();
        bestID = -1;
        curID = -1;
        best = -1;
        global_round++;
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
                        unsafe_intervals.emplace_back(constraint.tl, constraint.tr, -1);
                    }
                }

                std::sort(unsafe_intervals.begin(), unsafe_intervals.end());

                std::vector<Time_interval> safe_intervals;
                Time last_unsafe_end = -1;
                int current_key = 0;
                for (const auto& unsafe_interval : unsafe_intervals) {
                    if (unsafe_interval.start > last_unsafe_end + 1) {
                        safe_intervals.push_back({last_unsafe_end + 1, unsafe_interval.start - 1, current_key++});
                    }
                    last_unsafe_end = std::max(last_unsafe_end, unsafe_interval.end);
                }

                if (last_unsafe_end < max_time -1) {
                    safe_intervals.push_back({last_unsafe_end + 1, max_time -1, current_key++});
                }
                else if (unsafe_intervals.empty()) {
                    safe_intervals.push_back({0, max_time -1, current_key++});
                }
                all_safe_intervals[c_id] = safe_intervals;
            }
        }
    }

    inline const Node &cur() const { return this->nodes.at(curID); }



    Cost run(vid sx, vid sy, Time agent_available_at_t, STStateTracker& tracker) {
        init_search();

		// The priority_queue only store index of the data,
		// So we need a customized comparetor
        auto pcmp = [&](const ID &i, const ID &j) {
            return this->nodes[i] < this->nodes[j];
        };
        priority_queue<int, vector<int>, decltype(pcmp)> q(pcmp);

        best = bestID = -1;
        vid start_id = sy * width + sx;

        auto it_start_intervals = all_safe_intervals.find(start_id);
        if (it_start_intervals == all_safe_intervals.end() || it_start_intervals->second.empty()) {
            return -1;
        }
        const std::vector<Time_interval>& safe_intervals = it_start_intervals->second;
        for (const auto& interval : safe_intervals) {
          if (interval.end < agent_available_at_t) {
            continue;
          }
          Time start_time = std::max(agent_available_at_t, interval.start);
          q.push(gen_node(sx, sy, interval, start_time, hVal(sx, sy, start_time, tracker), start_time));
          //state_g_values[{sx, sy, interval}] = interval.start;
          gtable[id(sx, sy)][interval.key] = {start_time, global_round};
        }

        while (!q.empty()) {
          curID = q.top();
          q.pop();
          auto current_target = tracker.getCoordinatesAtTime(cur().arrival_time);
          if (cur().isAt(current_target.first, current_target.second)) {
            best = cur().g;
            bestID = curID;
            break;
          }
          //auto cur_it = state_g_values.find({cur().state.x, cur().state.y, cur().state.interval});
          //if(cur_it != state_g_values.end() && cur().arrival_time >= cur_it->second) {
          //  continue;
          //}
          if(gval(id(cur().state.x, cur().state.y), cur().state.interval.key) < cur().arrival_time)
            continue;

          
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
                    //auto it = state_g_values.find({nx, ny, interval});
                    //if(it != state_g_values.end() && new_arrival_time >= it->second) {
                    //    continue;
                    //}
                    if(gval(id(nx, ny), interval.key) <= new_arrival_time) {
                        continue;
                    }
                    ID nid = gen_node(nx, ny, interval, new_arrival_time, hVal(nx, ny, new_arrival_time, tracker), new_arrival_time);
                    gtable[id(nx, ny)][interval.key] = {new_arrival_time, global_round};
                    q.push(nid);
                    parent[nid] = curID;
                    //state_g_values[{nx, ny, interval}] = new_arrival_time;
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
        std::reverse(path.begin(), path.end()); 
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


