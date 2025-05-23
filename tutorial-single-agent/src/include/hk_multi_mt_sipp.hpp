#pragma once
#include <vector>
#include <limits>
#include <algorithm>
#include <iostream> 
#include <fstream> 
#include <string>
#include <format> 
#include <filesystem>


#include "SIPP.hpp"
#include "dynscens.hpp"
#include "gridmap.hpp"
#include "moving_target.hpp"
#include "mt_sipp.hpp"

// Entry for the Dynamic Programming (DP) table
struct DPEntry {
    Time time = std::numeric_limits<Time>::max() / 2; 
    vid x = -1;                       
    vid y = -1;                        
    int prev_target_idx = -1;                       
    int prev_mask = 0;                          
};

// Stores results of multi-target interception
struct MultiTargetResult {
    Time total_time = -1;                           
    std::vector<int> interception_order;             
    std::vector<mt_SIPP::STState> full_path;      
    bool success = false;                        
    std::vector<mt_SIPP::STState> actual_interception_events; 
};

class MultiTargetInterceptor {
public:
    mt_SIPP sipp_solver;           
    std::vector<STStateTracker>& target_trackers_ref; 
    const gridmap& g_map_ref;           
    const dynenv::NodeCSTRs& cstrs_ref;            
    int map_width;
    int map_height;

    MultiTargetInterceptor(
        const gridmap& g,
        const dynenv::NodeCSTRs& cs,
        int w, int h,
        std::vector<STStateTracker>& trackers)
        : sipp_solver(g, cs, w, h), target_trackers_ref(trackers),
          g_map_ref(g), cstrs_ref(cs), map_width(w), map_height(h) {
    }

    MultiTargetResult run_multi_moving_sipp(
        vid agent_start_x,
        vid agent_start_y,
        Time agent_initial_t) {

        int num_targets = target_trackers_ref.size();
        MultiTargetResult result;
        result.success = false; // Default to failure

        if (num_targets == 0) {
            result.total_time = agent_initial_t; 
            result.success = true;
            if (map_width > 0 && map_height > 0 && agent_start_x >=0 && agent_start_y >=0) {
                 result.full_path.push_back({agent_start_x, agent_start_y, agent_initial_t});
            }
            return result;
        }

        // DP Table: dp_table[mask][last_target_idx]
        std::vector<std::vector<DPEntry>> dp_table(
            1 << num_targets, 
            std::vector<DPEntry>(num_targets)
        );

        // 1. Initialization phase: agent start to each single target
        for (int i = 0; i < num_targets; ++i) {
            Time time_to_intercept_i = sipp_solver.run(
                agent_start_x, agent_start_y, agent_initial_t, target_trackers_ref[i]);

            if (time_to_intercept_i != -1 && time_to_intercept_i < sipp_solver.INFT) { 
                std::vector<mt_SIPP::STState> path_to_i = sipp_solver.get_path();
                if (!path_to_i.empty()) {
                    const auto& last_state = path_to_i.back(); 
                    int current_mask = 1 << i; 
                    dp_table[current_mask][i].time = time_to_intercept_i; 
                    dp_table[current_mask][i].x = last_state.x;
                    dp_table[current_mask][i].y = last_state.y;
                    dp_table[current_mask][i].prev_target_idx = -1; 
                    dp_table[current_mask][i].prev_mask = 0;       
                } else if (time_to_intercept_i == agent_initial_t) { 
                    // Agent might already be at the interception point at t0 for target i
                    int current_mask = 1 << i; 
                    dp_table[current_mask][i].time = time_to_intercept_i; 
                    dp_table[current_mask][i].x = agent_start_x; 
                    dp_table[current_mask][i].y = agent_start_y;
                    dp_table[current_mask][i].prev_target_idx = -1; 
                    dp_table[current_mask][i].prev_mask = 0; 
                }
            }
        }

        // 2. DP iteration: fill DP table
        for (int mask_val = 1; mask_val < (1 << num_targets); ++mask_val) { 
            for (int prev_target_idx = 0; prev_target_idx < num_targets; ++prev_target_idx) { 
                if (! (mask_val & (1 << prev_target_idx)) || dp_table[mask_val][prev_target_idx].time >= sipp_solver.INFT) {
                    continue;
                }
                Time time_after_prev = dp_table[mask_val][prev_target_idx].time;
                vid x_after_prev = dp_table[mask_val][prev_target_idx].x;
                vid y_after_prev = dp_table[mask_val][prev_target_idx].y;

                for (int next_target_idx = 0; next_target_idx < num_targets; ++next_target_idx) {
                    if (mask_val & (1 << next_target_idx)) { // If next_target already in current mask_val, skip
                        continue;
                    }
                    Time time_to_intercept_next = sipp_solver.run(
                        x_after_prev, y_after_prev, time_after_prev, target_trackers_ref[next_target_idx]);

                    if (time_to_intercept_next != -1 && time_to_intercept_next < sipp_solver.INFT) { 
                        std::vector<mt_SIPP::STState> path_to_next = sipp_solver.get_path();
                        if (!path_to_next.empty()) {
                            const auto& last_state_next = path_to_next.back();
                            int new_mask = mask_val | (1 << next_target_idx); 

                            // Key DP update: if a shorter path to (new_mask, next_target_idx) is found
                            if (time_to_intercept_next < dp_table[new_mask][next_target_idx].time) {
                                dp_table[new_mask][next_target_idx].time = time_to_intercept_next;
                                dp_table[new_mask][next_target_idx].x = last_state_next.x;
                                dp_table[new_mask][next_target_idx].y = last_state_next.y;
                                dp_table[new_mask][next_target_idx].prev_target_idx = prev_target_idx;
                                dp_table[new_mask][next_target_idx].prev_mask = mask_val;
                            }
                        } else if (time_to_intercept_next == time_after_prev) { 
                             // Agent might already be at interception point for next_target, or waits there
                             int new_mask = mask_val | (1 << next_target_idx);
                             if (time_to_intercept_next < dp_table[new_mask][next_target_idx].time) {
                                dp_table[new_mask][next_target_idx].time = time_to_intercept_next;
                                dp_table[new_mask][next_target_idx].x = x_after_prev; 
                                dp_table[new_mask][next_target_idx].y = y_after_prev;
                                dp_table[new_mask][next_target_idx].prev_target_idx = prev_target_idx;
                                dp_table[new_mask][next_target_idx].prev_mask = mask_val;
                            }
                        }
                    }
                }
            }
        }

        // 3. Find final result from DP table
        Time min_total_time = sipp_solver.INFT;
        int last_target_in_sequence = -1;
        int final_mask = (1 << num_targets) - 1; 
        
        for (int i = 0; i < num_targets; ++i) {
            if (dp_table[final_mask][i].time < min_total_time) {
                min_total_time = dp_table[final_mask][i].time;
                last_target_in_sequence = i;
            }
        }

        if (last_target_in_sequence == -1 || min_total_time >= sipp_solver.INFT) {
            if (num_targets > 0) std::cerr << "Failed to find a valid sequence to intercept all targets." << std::endl;
            return result; 
        }

        result.total_time = min_total_time;
        result.success = true;

        // 4. Reconstruct optimal interception order
        std::vector<int> order_reversed;
        int current_target_for_reconstruction = last_target_in_sequence;
        int current_mask_for_reconstruction = final_mask;

        while (current_mask_for_reconstruction != 0 && current_target_for_reconstruction != -1) {
            order_reversed.push_back(current_target_for_reconstruction);
            const DPEntry& entry = dp_table[current_mask_for_reconstruction][current_target_for_reconstruction];
            current_target_for_reconstruction = entry.prev_target_idx;
            current_mask_for_reconstruction = entry.prev_mask;
        }
         if (num_targets == 1 && order_reversed.empty() && last_target_in_sequence != -1){ 
             order_reversed.push_back(last_target_in_sequence);
         }
        result.interception_order.assign(order_reversed.rbegin(), order_reversed.rend());


        // 5. Reconstruct full path and record interception events
        std::vector<mt_SIPP::STState> final_path_segments;
        Time current_agent_time = agent_initial_t;
        vid current_agent_x = agent_start_x;
        vid current_agent_y = agent_start_y;
        
        result.actual_interception_events.clear(); 
        result.actual_interception_events.reserve(result.interception_order.size());

        for (int target_idx_in_order : result.interception_order) {
            if (target_idx_in_order < 0 || target_idx_in_order >= num_targets) {
                std::cerr << "Path reconstruction error: Invalid target index " << target_idx_in_order << std::endl;
                result.success = false; result.full_path.clear(); result.actual_interception_events.clear(); return result;
            }
            Time intercept_time_segment = sipp_solver.run(
                current_agent_x, current_agent_y, current_agent_time, target_trackers_ref[target_idx_in_order]);

            if (intercept_time_segment == -1 || intercept_time_segment >= sipp_solver.INFT) {
                std::cerr << "Path reconstruction error: SIPP failed to plan to target " << target_idx_in_order << " from (" 
                          << current_agent_x << "," << current_agent_y << "@" << current_agent_time << ")" << std::endl;
                result.success = false; result.full_path.clear(); result.actual_interception_events.clear(); return result;
            }

            std::vector<mt_SIPP::STState> segment_path = sipp_solver.get_path();
            
            // Record interception event
            if (!segment_path.empty()) {
                result.actual_interception_events.push_back(segment_path.back());
            } else { 
                result.actual_interception_events.push_back({current_agent_x, current_agent_y, intercept_time_segment});
            }

            bool segment_is_wait_at_location = false; 
            if (segment_path.empty()){
                // Logic to correctly build full_path, separate from recording intercept event
                if (intercept_time_segment == current_agent_time && 
                    ( (result.interception_order.size() == 1 && 
                       dp_table[1<<target_idx_in_order][target_idx_in_order].x == current_agent_x &&
                       dp_table[1<<target_idx_in_order][target_idx_in_order].y == current_agent_y ) ||
                      (!result.actual_interception_events.empty() && 
                       result.actual_interception_events.back().x == current_agent_x &&
                       result.actual_interception_events.back().y == current_agent_y) 
                    )
                  )
                {
                    segment_is_wait_at_location = true; 
                    segment_path.push_back({current_agent_x, current_agent_y, intercept_time_segment}); 
                }
                 else if (intercept_time_segment > current_agent_time) { 
                     segment_is_wait_at_location = true;
                     for(Time t_wait = current_agent_time +1; t_wait < intercept_time_segment; ++t_wait) { 
                        if (final_path_segments.empty() || !(final_path_segments.back().x == current_agent_x && final_path_segments.back().y == current_agent_y && final_path_segments.back().t == t_wait -1) ) {
                           if (!final_path_segments.empty() && final_path_segments.back().t >= t_wait) {} 
                           else final_path_segments.push_back({current_agent_x, current_agent_y, t_wait});
                        }
                     }
                     segment_path.push_back({current_agent_x, current_agent_y, intercept_time_segment});
                } else if (intercept_time_segment == current_agent_time) {
                     segment_is_wait_at_location = true;
                     segment_path.push_back({current_agent_x, current_agent_y, current_agent_time});
                }
                 else {
                    std::cerr << "Path reconstruction error: SIPP returned empty path for target " << target_idx_in_order << " but time or state mismatch." << std::endl;
                    result.success = false; result.full_path.clear(); result.actual_interception_events.clear(); return result;
                }
            }

            // Path concatenation logic
            if (!final_path_segments.empty() && !segment_path.empty()) {
                if (final_path_segments.back().x == segment_path.front().x &&
                    final_path_segments.back().y == segment_path.front().y &&
                    final_path_segments.back().t == segment_path.front().t) {
                    final_path_segments.insert(final_path_segments.end(), segment_path.begin() + 1, segment_path.end());
                } else if (final_path_segments.back().t < segment_path.front().t) { 
                    final_path_segments.insert(final_path_segments.end(), segment_path.begin(), segment_path.end());
                } else if (segment_is_wait_at_location && final_path_segments.back().t == segment_path.front().t) {
                     if(segment_path.size() > 1) { 
                        final_path_segments.insert(final_path_segments.end(), segment_path.begin() + 1, segment_path.end());
                     } 
                }
                 else { 
                     final_path_segments.insert(final_path_segments.end(), segment_path.begin(), segment_path.end());
                }

            } else if (!segment_path.empty()) { 
                 final_path_segments.insert(final_path_segments.end(), segment_path.begin(), segment_path.end());
            }
            
            // Update agent state
            if (!final_path_segments.empty()){
                current_agent_time = final_path_segments.back().t;
                current_agent_x = final_path_segments.back().x;
                current_agent_y = final_path_segments.back().y;
            } else if (num_targets > 0) { 
                std::cerr << "Path reconstruction error: Cannot update agent state after empty path segment (num_targets=" << num_targets << ")" << std::endl;
                result.success = false; result.full_path.clear(); result.actual_interception_events.clear(); return result;
            }
        }
        result.full_path = final_path_segments;

        return result;
    }
};