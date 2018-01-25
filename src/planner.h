#pragma once

#include "map.h"
#include "geo.h"
#include "sensors.h"
#include "udacity.h"
#include <spline.h>
#include <json.hpp>
#include <utility>
#include <limits>
#include <algorithm>
#include <vector>
#include <tuple>
#include <stdexcept>
#include <cassert>
#include "ppl_planner.h"

namespace pp {

    // ego position with history
    struct ref_ego_position {
        ego_position ego;
        
        // previous x, y positions
        double prev_x, prev_y;

        // reference lane
        int lane_id;

        // number of required path points
        std::size_t path_point_nr;
    };

    constexpr static std::size_t c_path_point_nr = 50u;

    inline bool test_eq(ref_ego_position const& ref_, pp_l::PathPlanner const& pl_)
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

    class planner {
        pp::map _map;

        mutable pp_l::PathPlanner _planner;

        ref_ego_position ref;
    public:
        // Estimate reference position based on current and past data. The estimations are from the lessons and from Q/A walk-through. 
        static ref_ego_position get_ref_ego_position(pp::telemetry_data const& t_, double dt_) 
        {
            ref_ego_position ref;

            if (t_.previous_path.size() < 2u) {
                // first time calculation
                ref.ego = t_.ego;
                ref.prev_x = ref.ego.x - std::cos(ref.ego.yaw);
                ref.prev_y = ref.ego.y - std::sin(ref.ego.yaw);
            } else {
                // standard case, path calculated before
                ref.ego.x = *(t_.previous_path.x.end() - 1);
                ref.ego.y = *(t_.previous_path.y.end() - 1);

                ref.prev_x = *(t_.previous_path.x.end() - 2);
                ref.prev_y = *(t_.previous_path.y.end() - 2);

                ref.ego.yaw = std::atan2(ref.ego.y - ref.prev_y, ref.ego.x - ref.prev_x);
                ref.ego.speed = pp::edistance(ref.prev_x, ref.prev_y, ref.ego.x, ref.ego.y) / dt_;

                ref.ego.s = t_.end_path.s;
                ref.ego.d = t_.end_path.d;
            }

            ref.lane_id = pp_l::Track{}.lane_at(ref.ego.d);

            ref.path_point_nr =
                std::max(pp::c_path_point_nr, t_.previous_path.size()) -
                std::min(pp::c_path_point_nr, t_.previous_path.size());

            return ref;
        }
    public:
        planner(pp::map map_)
            : _map(map_)
        {
            //_planner.initialize(_map.get_filename());
            _planner.initialize(_map);
            _planner.reset();
        }

        auto get_next_vals(pp::telemetry_data const& t_, double dt_) const
        {
            //std::cout
            //    << __FUNCTION__ << std::endl
            //    << "Time difference: " << dt_ << std::endl;
            
            pp::path path;
            {
                _planner.compute_reference(t_, dt_);
                
                {
                    auto ref = get_ref_ego_position(t_, dt_);
                    assert(test_eq(ref, _planner));
                }

                _planner.track_lap(t_);
                _planner.process_sensor_fusion(t_, dt_);
                _planner.create_plan(t_, dt_);
                _planner.collision_avoidance();
                _planner.speed_control();
                _planner.build_path(t_, _planner.target_lane, _planner.target_speed, path, dt_);
            }
            //_planner.run(t_, path, dt_);

            return std::make_pair(path.x, path.y);
        }
    };
}