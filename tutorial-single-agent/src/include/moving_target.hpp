#pragma once

#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include <sstream>
#include <algorithm>
#include <utility>
#include <algorithm>
#include <cassert>
#include <limits>
#include <math.h>
#include <queue>
#include <set>
#include <tuple>
#include <map>
#include "gridmap.hpp"
#include "dynscens.hpp"
using namespace std;
using namespace movingai;

using vid = int;
using Time = int;

struct STState {
    vid x, y;
    Time t;

    STState(vid x_val, vid y_val, Time t_val) : x(x_val), y(y_val), t(t_val) {}
};

class STStateTracker {
public:
    std::vector<STState> states;

    STStateTracker() = default;

    bool loadStatesFromFile(const std::string& filename) {
        std::ifstream infile(filename);
        if (!infile.is_open()) {
            std::cerr << "Error: Could not open file " << filename << std::endl;
            return false;
        }

        states.clear();
        std::string line;
        vid x_val, y_val;
        Time t_val;

        while (std::getline(infile, line)) {
            std::istringstream iss(line);
            if (!(iss >> x_val >> y_val >> t_val)) {
                std::cerr << "Error: Invalid line format in file: " << line << std::endl;
                continue;
            }
            states.emplace_back(x_val, y_val, t_val);
        }

        infile.close();

        // std::sort(states.begin(), states.end(), [](const STState& a, const STState& b) {
        //     return a.t < b.t;
        // });

        return true;
    }

    std::pair<vid, vid> getCoordinatesAtTime(Time t_input) const {
        if (states.empty()) {
            std::cerr << "Error: No states loaded." << std::endl;
            return std::make_pair(-1, -1);
        }

        if (t_input >= states.back().t) {
            return std::make_pair(states.back().x, states.back().y);
        }

        if (t_input < states.front().t) {
            return std::make_pair(states.front().x, states.front().y);
        }

        for (size_t i = states.size() - 1; i > 0; --i) {
            if (states[i-1].t <= t_input && states[i].t > t_input) {
                return std::make_pair(states[i-1].x, states[i-1].y);
            }
        }
        return std::make_pair(states.front().x, states.front().y);

        /*
        auto it = std::upper_bound(states.begin(), states.end(), t_input,
            [](Time val, const STState& state) {
                return val < state.t;
            });

        if (it == states.begin()) {
            return std::make_pair(states.front().x, states.front().y);
        }
        --it;
        return std::make_pair(it->x, it->y);
        */
    }

    void printStates() const {
        if (states.empty()) {
            std::cout << "No states to print." << std::endl;
            return;
        }
        std::cout << "Loaded States (x, y, t):" << std::endl;
        for (const auto& state : states) {
            std::cout << state.x << " " << state.y << " " << state.t << std::endl;
        }
    }

    int getMinDistanceToPoint(vid target_x, vid target_y, Time t) const {
        if (states.empty()) {
            return std::numeric_limits<int>::max();
        }

        int min_distance = std::numeric_limits<int>::max();

        for (const auto& state : states) {
            if (state.t <= t) {
                continue;
            }
            int diff_x = state.x - target_x;
            int abs_diff_x = std::abs(diff_x);

            int diff_y = state.y - target_y;
            int abs_diff_y = std::abs(diff_y);
            
            int current_distance = abs_diff_x + abs_diff_y;

            if (current_distance < min_distance) {
                min_distance = current_distance;
            }
        }
        return min_distance;
    }
};
