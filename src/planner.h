#pragma once

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
    // ego position with history
    struct ref_ego_position {
        ego_position ego;
        
        // previous x, y positions
        double prev_x, prev_y;

        // reference lane
        int lane_id;

        // number of required path points
        std::size_t path_point_count;
    };

    struct lane_info {
        struct car {
            int id = -1;
            double gap = pp::c_inf;
            double speed = pp::c_inf;
        };

        static bool exists(std::pair<bool, car> const& c_)
        {
            return c_.first;
        }

        static int id(std::pair<bool, car> const& c_)
        {
            return c_.second.id;
        }

        static double& gap(std::pair<bool, car>& c_)
        {
            return c_.second.gap;
        }

        static double const& gap(std::pair<bool, car> const& c_)
        {
            return c_.second.gap;
        }

        static double& speed(std::pair<bool, car>& c_)
        {
            return c_.second.speed;
        }

        static double const& speed(std::pair<bool, car> const& c_)
        {
            return c_.second.speed;
        }

        bool is_feasible() const
        {
            return
                (!lane_info::exists(front) || 
                pp::c_lane_change_min_front_buffer < lane_info::gap(front)) &&
                (!lane_info::exists(back) ||
                pp::c_lane_change_min_back_buffer < lane_info::gap(back));
        }

        std::pair<bool, car> 
            front = {false, car{-1, c_inf, c_inf}}, 
            back = {false, car{-1, c_inf, 0.}};

        friend std::ostream& operator<<(std::ostream& out_, lane_info const& l_)
        {
            return out_
                << "f: " << std::boolalpha << lane_info::exists(l_.front)
                << ", " << lane_info::id(l_.front)
                << ", " << lane_info::gap(l_.front)
                << ", " << lane_info::speed(l_.front)
                << std::endl
                << "b: " << std::boolalpha << lane_info::exists(l_.back)
                << ", " << lane_info::id(l_.back)
                << ", " << lane_info::gap(l_.back)
                << ", " << lane_info::speed(l_.back)
                << std::endl
                << "feasible: " << std::boolalpha << l_.is_feasible();
        }
    };
 

    // auxiliary calculations for getting the center or width of lanes
    // Based on the assumption that lane_id are integers in [0, n - 1] where
    // n is pp::c_lane_count
    struct road {
        static constexpr auto c_width = pp::c_lane_count * pp::c_lane_width;

        // Lane center from the origin (d)
        static double get_lane_center_d(int lane_id_)
        {
            return (0.5 + lane_id_) * pp::c_lane_width;
        }

        // Lane center with safety margin in border lanes 
        static double get_safe_lane_center_d(int lane_id_)
        {
            auto c = get_lane_center_d(lane_id_);

            if (lane_id_ == 0)
                c += c_lane_dist_safety_margin;
            else if (lane_id_ == pp::c_lane_count - 1)
                c -= c_lane_dist_safety_margin;
            
            return c;
        }

        static int get_lane_id(pp::fpoint const & p_)
        {
            return get_lane_id(p_.d);
        }

        // Lane at a given distance from the road center
        static int get_lane_id(double d_)
        {
            return std::floor(d_ / pp::c_lane_width);
        }

        static double get_distance_to_lane_center_d(double d_, int lane_id_)
        {
            return get_lane_center_d(lane_id_) - d_;
        }
    };

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

    inline bool test_eq(std::vector<lane_info> const& lane_env_model, pp_l::PathPlanner const& pl_)
    {
        if (lane_env_model.size() == pl_.lane_info.size()) {
            return std::equal(
                lane_env_model.begin(), lane_env_model.end(),
                pl_.lane_info.begin(), pl_.lane_info.end(),
                [](pp::lane_info const& l_, pp_l::lane_info_t const & r_) {
                return
                    lane_info::exists(l_.front) ? r_.front_car != -1 : r_.front_car == -1 &&
                    lane_info::id(l_.front) == r_.front_car &&
                    lane_info::gap(l_.front) == r_.front_gap &&
                    lane_info::speed(l_.front) == r_.front_speed &&
                    lane_info::exists(l_.back) ? r_.back_car != -1 : r_.back_car == -1 &&
                    lane_info::id(l_.back) == r_.back_car &&
                    lane_info::gap(l_.back) == r_.back_gap &&
                    lane_info::speed(l_.back) == r_.back_speed &&
                    l_.is_feasible() == r_.feasible;
            });
        } else {
            return false;
        }
    }

    class planner {
        pp::map _map;

        mutable pp_l::PathPlanner _planner;
    public:
        // Estimate reference position based on current and past data. The estimations are from the lessons and from Q/A walk-through. 
        static ref_ego_position make_ref_ego_position(pp::telemetry_data const& t_, double dt_) 
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

            ref.lane_id = road::get_lane_id(ref.ego.d);

            ref.path_point_count =
                std::max(pp::c_path_point_count, t_.previous_path.size()) -
                std::min(pp::c_path_point_count, t_.previous_path.size());

            return ref;
        }

        // Predict and remember closest cars on each lane
        static auto make_lane_model(
            ref_ego_position const & p_, 
            pp::telemetry_data const& t_, 
            double dt_)
        {
            std::vector<lane_info> lane_env_model{std::size_t{pp::c_lane_count}};

            // What a heck is this?
            auto pred_distance = t_.previous_path.size();

            for (auto & other : t_.sensor_fusion) {
                auto lane_id = road::get_lane_id(other.d);
                
                // car is on our side of the road
                if (0 <= lane_id && lane_id < pp::c_lane_count) {
                    auto& l = lane_env_model[lane_id];

                    // estimate positions based on sensor data
                    auto other_speed = pp::enorm(other.vx, other.vy);
                    auto other_s = other.s + other_speed * pred_distance * dt_;
                    auto other_gap = std::fabs(other_s - p_.ego.s);

                    // Is other car in front of me and closer than this one?
                    if (p_.ego.s <= other_s && other_gap < lane_info::gap(l.front)) {                        
                        l.front = std::make_pair(true, lane_info::car{
                            other.uid,
                            other_gap,
                            other_speed
                        });
                    }
                    
                    // Is the other car behind me and closer than minimal horizon...
                    if (other_s < p_.ego.s && other_gap < std::min(pp::c_lane_min_horizon, lane_info::gap(l.back))) {
                        l.back = std::make_pair(true, lane_info::car{
                            other.uid,
                            other_gap,
                            other_speed
                        });
                    }
                };
            }

            return lane_env_model;
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
                
                auto ref_ego_pos = make_ref_ego_position(t_, dt_);
                assert(test_eq(ref_ego_pos, _planner));

                _planner.track_lap(t_); 
                _planner.process_sensor_fusion(t_, dt_);
                
                auto lane_model = planner::make_lane_model(ref_ego_pos, t_, dt_); 
                assert(test_eq(lane_model, _planner));

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