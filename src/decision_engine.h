#pragma once

#include "localization.h"
#include "map.h"
#include "road.h"
#include <tuple>
#include <vector>

namespace pp {

    enum class engine_states : int {
        start = 0,
        keeping_lane,
        prepare_changing_lane,
        changing_lane
    };

    struct current_vals {
        // current state
        engine_states state;

        // trajectory driven in current state
        double s;
    };

    struct target_vals {
        int lane_id = 1;
        double speed = 0.; 
        int changing_into_lane_id = -1; 
    };

    class decision_engine {
        // current state:
        current_vals _current = {engine_states::start, 0.}; 
        // targets in current state
        target_vals _target;

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
                    !std::get<2>(li_).is_clear() && std::get<1>(li_) < pp::c_inf;
            }), bli.end());

            // sort lanes by distance to reference lane
            std::sort(bli.begin(), bli.end(), [&](auto const& l_, auto const& r_) {
                return abs(std::get<0>(l_) - ref_.lane_id) < abs(std::get<0>(l_) - ref_.lane_id);
            });

            return !bli.empty() ? std::get<0>(bli.front()) : ref_.lane_id;
        }

        void switch_state(engine_states new_state_, localization_info const& ref_)
        {
            if (_current.state != new_state_) {
                // Set the new state and the reference position
                _current.state = new_state_;
                _current.s = ref_.ego.s;
            }
        }
    public:
        current_vals const& get_current() const
        {
            return _current;
        }

        target_vals const& get_target() const
        {
            return _target;
        }

        void next_target(
            pp::map const& map_,
            localization_info const& ref_, 
            std::vector<lane_info> const li_)
        {
            std::cout << "next decision" << std::endl;

            // Give it a safety margin
            const auto c_max_speed = pp::mph2mps(pp::c_speed_limit_mph) - pp::c_acceleration;

            if (_current.state == engine_states::start)
            {
                _target.changing_into_lane_id = -1;
                _target.lane_id = ref_.lane_id;
                _target.speed = 0.;
                _current.state = engine_states::keeping_lane;
                _current.s = ref_.ego.s;
            }

            int best_lane = get_best_lane_id(ref_, li_);
            std::cout << " - best lane " << best_lane << std::endl;

            while (true) {
                double meters_in_state = map_.normalize_s(ref_.ego.s - _current.s);
                
                if (_current.state == engine_states::keeping_lane) {
                    assert(_target.lane_id == ref_.lane_id);

                    std::cout 
                        << " - keeping lane " << _target.lane_id
                        << " for " << std::setprecision(2) << pp::meters2miles(meters_in_state) << " Miles"
                        << " (cte=" << std::setprecision(1) << std::setw(4) << get_cte(ref_) << " m)"
                        << std::endl;

                    _target.speed = c_max_speed;

                    // Evaluate lane change
                    _target.changing_into_lane_id = -1;

                    // Evaluate lane changes if current lane is busy
                    if (lane_info::gap(li_[_target.lane_id].front) < pp::c_lane_min_horizon && 
                        lane_info::speed(li_[_target.lane_id].front) < c_max_speed) {
                        // Change if we are not in the best one
                        if (lane_info::speed(li_[_target.lane_id].front) + pp::c_acceleration < lane_info::speed(li_[best_lane].front)) {
                            _target.changing_into_lane_id = best_lane;
                            switch_state(engine_states::prepare_changing_lane, ref_);
                            continue;
                        }
                    }

                    break;
                } else if (_current.state == engine_states::prepare_changing_lane) {
                    assert(_target.changing_into_lane_id != ref_.lane_id);

                    std::cout << " - preparing lane change " << _target.changing_into_lane_id
                        << " for " << std::setprecision(2) << pp::meters2miles(meters_in_state) << " Miles"
                        << " (cte=" << std::setprecision(1) << std::setw(4) << get_cte(ref_) << " m)"
                        << std::endl;

                    _target.lane_id = ref_.lane_id + ((_target.changing_into_lane_id > ref_.lane_id) ? 1 : -1);

                    if (li_[_target.lane_id].is_feasible()) {
                        switch_state(engine_states::changing_lane, ref_);
                        continue;
                    } else if (_target.changing_into_lane_id != best_lane || meters_in_state > 500) {
                        // Waiting too much or not the best: cancel
                        _target.lane_id = ref_.lane_id;
                        switch_state(engine_states::keeping_lane, ref_);
                        continue;
                    } else { // Not feasible
                        if (lane_info::gap(li_[ref_.lane_id].front) < 30) {
                            // Can't perform change, try slow down:
                            if (lane_info::gap(li_[_target.lane_id].back) < pp::c_lane_change_min_back_buffer)
                                _target.speed = 
                                std::min(_target.speed, lane_info::speed(li_[_target.lane_id].back) - 0.5);
                            else
                                _target.speed = 
                                std::min(_target.speed, lane_info::speed(li_[_target.lane_id].front) - 0.5);
                        }
                        // But wait in this lane
                        _target.lane_id = ref_.lane_id;
                    }

                    break;
                } else if (_current.state == engine_states::changing_lane) {
                    if (ref_.lane_id == _target.lane_id && 
                        std::fabs(get_cte(ref_)) < 0.2 && 
                        100 < meters_in_state) {
                        // Lane change completed
                        if (_target.changing_into_lane_id >= 0 && _target.changing_into_lane_id != ref_.lane_id) {
                            switch_state(engine_states::prepare_changing_lane, ref_);
                            continue;
                        }

                        _target.changing_into_lane_id = -1;
                        switch_state(engine_states::keeping_lane, ref_);
                        continue;
                    }

                    if (std::fmin(lane_info::gap(li_[_target.lane_id].front), lane_info::gap(li_[_target.lane_id].back)) < 5.) {
                        // abort too dangerous...
                        _target.lane_id = ref_.lane_id;
                        _target.changing_into_lane_id = -1;
                        std::cout << " - aborting lane change" << std::endl;
                    }

                    std::cout << " - changing to lane " << _target.lane_id
                        << " for " << std::setprecision(2) << pp::meters2miles(meters_in_state) << " Miles"
                        << " (error=" << std::setprecision(1) << std::setw(4) << get_cte(ref_) << " m)"
                        << std::endl;

                    _target.speed = c_max_speed;
                    break;
                }

                break;
            }

            // Clamp speed:
            _target.speed = std::max(0., std::min(c_max_speed, _target.speed));
        
            std::cout 
                << " - changing lane " << _target.changing_into_lane_id << std::endl
                << " - target lane   " << _target.lane_id << std::endl
                << " - target speed  " << std::setprecision(2) << std::fixed
                << pp::mps2mph(_target.speed) << std::endl;
        }
    };
}