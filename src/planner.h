#pragma once

#include "decision_engine.h"
#include "localization.h"
#include "road.h"
#include "constants.h"
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

    class planner {
        pp::map _map;
        
        mutable pp_l::PathPlanner _planner;
    public:
       
        decision_engine _engine;

        current_vals _current;
        target_vals _target;
    public:
        planner(pp::map map_)
            : _map(map_)
        {
            _planner.initialize(_map);
            _planner.reset();
        }

        auto get_next_vals(pp::telemetry_data const& t_, double dt_)
        {
            //std::cout
            //    << __FUNCTION__ << std::endl
            //    << "Time difference: " << dt_ << std::endl;
            
            pp::path path;
            {
                _planner.compute_reference(t_, dt_);
                
                auto loc = pp::localize(t_, dt_);
                assert(tests::test_eq(loc, _planner));

                _planner.track_lap(t_); 
                _planner.process_sensor_fusion(t_, dt_);
               
                auto lane_model = pp::make_lane_model(loc.ego.s, t_, dt_); 
                assert(tests::test_eq(lane_model, _planner));

                _planner.create_plan(t_, dt_);

                _engine.next_target(_map, loc, lane_model);
                _current = _engine.get_current();
                _target = _engine.get_target();
                assert(tests::test_eq(_engine, _planner));

                _planner.collision_avoidance();
               
                avoid_collision(lane_model);
                assert(_target.speed == _planner.target_speed);

                _planner.speed_control();
                adjust_speed(loc);
                assert(_target.speed == _planner.target_speed);

                _planner.build_path(t_, _planner.target_lane, _planner.target_speed, path, dt_);

            }

            return std::make_pair(path.x, path.y);
        }

        // Modify speed to reduce chance for collision
        void avoid_collision(std::vector<lane_info> const& li_)
        {
            if (lane_info::gap(li_[_target.lane_id].front) < 30.) {
                if (lane_info::gap(li_[_target.lane_id].front) < 15.)
                    _target.speed = std::max(0., std::min(_target.speed, lane_info::speed(li_[_target.lane_id].front) - pp::c_acceleration));
                else
                    _target.speed = std::min(_target.speed, lane_info::speed(li_[_target.lane_id].front));

                std::cout
                    << " - reducing target speed to " << pp::mps2mph(_target.speed) << " mph "
                    << "in order to keep safety gap to lead of "
                    << lane_info::gap(li_[_target.lane_id].front) << " m"
                    << std::endl;
            }
        }

        void adjust_speed(localization_info const& ref_)
        {
            if (_target.speed < ref_.ego.speed) {
                // decelerate
                _target.speed = std::max(_target.speed, ref_.ego.speed - pp::c_acceleration);
            } else if (ref_.ego.speed < _target.speed) {
                // accelerate
                _target.speed = std::min(_target.speed, ref_.ego.speed + pp::c_acceleration);
            }
        }

        tk::spline make_anchors(map const& map_, localization_info const& ref_) const
        {
            auto const target_d = road::get_safe_lane_center_d(_target.lane_id);

            std::cout << "trajectory" << std::endl
                << " - target lane " << _target.lane_id << std::endl
                << " - target speed " << std::setprecision(1) << pp::mps2mph(_target.speed) << std::endl
                << " - target d " << std::setprecision(2) << target_d << std::endl;

            // trajectory points
            pp::path anchors;

            // Build a path tangent to the previous end states
            anchors.append({ref_.prev_x, ref_.prev_y});
            anchors.append({ref_.ego.x, ref_.ego.y});

            // Add 3 more points spaced 30 m
            for (int i = 1; i <= 3; i++) {
                anchors.append(map_.get_xy(ref_.ego.s + 30 * i, target_d));
            }

            // change the points to the reference (ref_) coordinate
            for (int i = 0; i < anchors.size(); i++) {
                const auto dx = anchors.x[i] - ref_.ego.x;
                const auto dy = anchors.y[i] - ref_.ego.y;
                anchors.x[i] = dx * std::cos(-ref_.ego.yaw) - dy * std::sin(-ref_.ego.yaw);
                anchors.y[i] = dx * std::sin(-ref_.ego.yaw) + dy * std::cos(-ref_.ego.yaw);
            }

            // Interpolate the anchors with a cubic spline
            tk::spline spl;
            spl.set_points(anchors.x, anchors.y);

            return std::move(spl);
        }


        auto make_trajectory(map const& map_, localization_info const& ref_, telemetry_data const& t_, double dt_) const
        {
            // Anchors as spline
            auto spl_path = make_anchors(map_, ref_);

            // Add previous path for continuity
            pp::path path{
                t_.previous_path.x,
                t_.previous_path.y
            };

            // set a horizon of 30 m
            auto const target_x = 30;
            auto const target_y = spl_path(target_x);
            auto const target_dist = pp::enorm(target_x, target_y);
            auto const t = target_x / target_dist * dt_;

            // sample the spline curve to reach the target speed
            for (auto i = 1; i <= pp::c_path_point_count - path.x.size(); i++) {
                auto x = i * t * _target.speed;
                auto y = spl_path(x);
                // transform back to world coordinates
                auto xw = x * std::cos(ref_.ego.yaw) - y * sin(ref_.ego.yaw) + ref_.ego.x;
                auto yw = x * std::sin(ref_.ego.yaw) + y * cos(ref_.ego.yaw) + ref_.ego.y;
                // append the trajectory points
                path.append({xw, yw});
            }

            return std::make_pair(std::move(path.x), std::move(path.y));
        }
    };
}