#pragma once

#include "road.h"
#include "sensors.h"
#include <cstddef>
#include <cassert>

#include "ppl_planner.h"

namespace pp {
    // ego position with history
    struct localization_info {
        ego_position ego;

        // previous x, y positions
        double prev_x, prev_y;

        // reference lane
        int lane_id;

        // number of required path points
        std::size_t path_point_count;
    };

    // Estimate reference position based on current and past data. The estimations are from the lessons and from Q/A walk-through. 
    inline localization_info localize(pp::telemetry_data const& t_, double dt_)
    {
        localization_info loc;

        if (t_.previous_path.size() < 2u) {
            // first time calculation
            loc.ego = t_.ego;
            loc.prev_x = loc.ego.x - std::cos(loc.ego.yaw);
            loc.prev_y = loc.ego.y - std::sin(loc.ego.yaw);
        } else {
            // standard case, path calculated before
            loc.ego.x = *(t_.previous_path.x.end() - 1);
            loc.ego.y = *(t_.previous_path.y.end() - 1);

            loc.prev_x = *(t_.previous_path.x.end() - 2);
            loc.prev_y = *(t_.previous_path.y.end() - 2);

            loc.ego.yaw = std::atan2(loc.ego.y - loc.prev_y, loc.ego.x - loc.prev_x);
            loc.ego.speed = pp::edistance(loc.prev_x, loc.prev_y, loc.ego.x, loc.ego.y) / dt_;

            loc.ego.s = t_.end_path.s;
            loc.ego.d = t_.end_path.d;
        }

        loc.lane_id = road::get_lane_id(loc.ego.d);

        loc.path_point_count =
            std::max(pp::c_path_point_count, t_.previous_path.size()) -
            std::min(pp::c_path_point_count, t_.previous_path.size());

        return loc;
    }

    namespace tests {
        inline bool test_eq(localization_info const& ref_, pp_l::PathPlanner const& pl_)
        {
            return
                ref_.ego.d == pl_.ref_d &&
                ref_.ego.s == pl_.ref_s &&
                ref_.ego.speed == pl_.ref_speed &&
                ref_.ego.x == pl_.ref_x &&
                ref_.ego.y == pl_.ref_y &&
                ref_.ego.yaw == pl_.ref_yaw &&
                ref_.lane_id == pl_.ref_lane &&
                ref_.prev_x == pl_.ref_x_prev &&
                ref_.prev_y == pl_.ref_y_prev;
        }
    }
}