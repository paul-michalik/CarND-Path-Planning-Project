#pragma once

#include "localization.h"
#include "map.h"
#include "road.h"
#include <tuple>
#include <vector>

#include "ppl_planner.h"

namespace pp {
    struct target_info {
        int lane_id = 1;
        double speed = 0.; 
        int changing_into_lane_id = -1;
    };

    struct decision_engine {
        enum class state {
            start,
            keeping_lane,
            prepare_changing_lane,
            changing_lane
        };

        state _cur_state = state::start;
        
        // trajectory driven in current state
        double _cur_state_s = 0.;
        target_info _target;

        auto get_cte(localization_info const& ref_) const
        {
            return ref_.ego.d - road::get_lane_center_d(_target.lane_id);
        }

        // estimate best lane (= discrete cost function)
        auto get_best_lane_id(localization_info const& ref_, std::vector<lane_info> const& li_) const
        {
            using best_lane_info_t = std::tuple<int, double, lane_info>;

            std::vector<best_lane_info_t> bli;
            {
                int lane_id = 0;
                std::transform(li_.begin(), li_.end(), std::back_inserter(bli), [&](lane_info const& l_) {
                    return std::make_tuple(lane_id++, l_.get_lane_speed(), l_);
                });
            }

            // pick only clear lanes <2> with infinite velocities <1>
            bli.erase(std::remove_if(bli.begin(), bli.end(), 
                [&](best_lane_info_t const& li_) {
                return
                    !std::get<2>(li_).is_feasible() &&
                    std::get<1>(li_) < pp::c_inf;
            }), bli.end());

            // sort lanes by distance to reference lane
            std::sort(bli.begin(), bli.end(), [&](auto const& l_, auto const& r_) {
                return abs(get<0>(l_) - ref_.lane_id) < abs(get<0>(l_) - ref_.lane_id);
            });

            return !bli.empty() ? std::get<0>(bli.front()) : ref_.lane_id;
        }

        void switch_state(state new_state_, localization_info const& ref_)
        {
            if (_cur_state != new_state_) {
                // Set the new state and the reference position
                _cur_state = new_state_;
                _cur_state_s = ref_.ego.s;
            }
        }
    public:
        target_info make_decision(
            pp::map const& map_,
            localization_info const& ref_, 
            std::vector<lane_info> const li_)
        {
            // Give it a safety margin
            const auto max_speed = pp::mph2mps(pp::c_speed_limit_mph) - 0.2;

            if (_cur_state == state::start)
            {
                _target.changing_into_lane_id = -1;
                _target.lane_id = ref_.lane_id;
                _target.speed = 0.;
                _cur_state = state::keeping_lane;
                _cur_state_s = ref_.ego.s;
            }

            int best_lane = get_best_lane_id(ref_, li_);
            std::cout << " - best lane " << best_lane << std::endl;

            while (true) {
                double meters_in_state = map_.normalize_s(ref_.ego.s - _cur_state_s);
                
                if (_cur_state == state::keeping_lane) {
                    assert(_target.lane_id == ref_.lane_id);

                    std::cout 
                        << " - keeping lane " << _target.lane_id
                        << " for " << std::setprecision(2) << pp::meters2miles(meters_in_state) << " Miles"
                        << " (cte=" << std::setprecision(1) << std::setw(4) << get_cte(ref_) << " m)"
                        << std::endl;

                    _target.speed = max_speed;

                    // Evaluate lane change
                    _target.changing_into_lane_id = -1;

                    // Evaluate lane changes if current lane is busy
                    if (lane_info::gap(li_[_target.lane_id].front) < pp::c_lane_min_horizon
                        && lane_info::speed(li_[_target.lane_id].front) < max_speed) {
                        // Change if we are not in the best one
                        if (lane_info::speed(li_[_target.lane_id].front) + 0.2 < lane_info::speed(li_[best_lane].front)) {
                            _target.changing_into_lane_id = best_lane;
                            switch_state(state::prepare_changing_lane, ref_);
                            continue;
                        }
                    }

                    break;
                }

            //    else if (state_ == STATE::PLC)
            //    {
            //        assert(changing_lane != ref_lane);

            //        std::cout << " * PREPARING CHANGE TO LANE " << changing_lane
            //            << " FOR " << setprecision(2) << pp::meters2miles(meters_in_state) << " Miles"
            //            << " (cte=" << setprecision(1) << setw(4) << cte << " m)"
            //            << endl;

            //        target_lane = ref_lane + ((changing_lane > ref_lane) ? 1 : -1);

            //        if (lane_info[target_lane].feasible)
            //        {
            //            set_state(t_, STATE::LC);
            //            continue;
            //        } else if (changing_lane != best_lane || meters_in_state > 500)
            //        {
            //            // Waiting too much or not the best: cancel
            //            target_lane = ref_lane;
            //            set_state(t_, STATE::KL);
            //            continue;
            //        } else // Not feasible
            //        {
            //            if (lane_info[ref_lane].front_gap < 30) {
            //                // Can't perform the change ... try to slow down here
            //                if (lane_info[target_lane].back_gap < lane_change_back_buffer)
            //                    target_speed = fmin(target_speed, lane_info[target_lane].back_speed - 0.5);
            //                else
            //                    target_speed = fmin(target_speed, lane_info[target_lane].front_speed - 0.5);
            //            }
            //            // But wait in this lane
            //            target_lane = ref_lane;
            //        }

            //        break;
            //    }

            //    else if (state_ == STATE::LC)
            //    {
            //        if (ref_lane == target_lane && fabs(cte) < 0.2 && meters_in_state > 100)
            //        {
            //            // Lane change completed
            //            if (changing_lane >= 0 && changing_lane != ref_lane) {
            //                set_state(t_, STATE::PLC);
            //                continue;
            //            }

            //            changing_lane = -1;
            //            set_state(t_, STATE::KL);
            //            continue;
            //        }

            //        if (fmin(lane_info[target_lane].front_gap, lane_info[target_lane].back_gap) < 5)
            //        {
            //            // ABORT
            //            target_lane = ref_lane;
            //            changing_lane = -1;
            //            std::cout << " * ABORTING LANE CHANGE" << endl;
            //        }

            //        std::cout << " * CHANGING TO LANE " << target_lane
            //            << " FOR " << setprecision(2) << pp::meters2miles(meters_in_state) << " Miles"
            //            << " (error=" << setprecision(1) << setw(4) << cte << " m)"
            //            << endl;

            //        target_speed = road_speed_limit;
            //        break;
            //    }

            //    break;
            }

            //// Ensure the target speed is inside the limits
            //// NOTE that we don't consider the possibility of moving backwards.
            //target_speed = fmax(0.0, fmin(road_speed_limit, target_speed));
        
            return _target;
        }
    };

    namespace tests {
        inline bool test_eq(decision_engine::state const& l_, pp_l::PathPlanner::STATE const& r_)
        {
            return
                l_ == decision_engine::state::changing_lane && r_ == pp_l::PathPlanner::STATE::LC ||
                l_ == decision_engine::state::keeping_lane && r_ == pp_l::PathPlanner::STATE::KL ||
                l_ == decision_engine::state::prepare_changing_lane && r_ == pp_l::PathPlanner::STATE::PLC ||
                l_ == decision_engine::state::start && r_ == pp_l::PathPlanner::STATE::START;
        }

        inline bool test_eq(decision_engine const& engine_, pp_l::PathPlanner const& pl_)
        {
            return test_eq(engine_._cur_state, pl_.state_) &&
                engine_._cur_state_s == pl_.state_s_ &&
                engine_._target.lane_id == pl_.target_lane &&
                engine_._target.speed == pl_.target_speed &&
                engine_._target.changing_into_lane_id == pl_.changing_lane;
        }
    }
}