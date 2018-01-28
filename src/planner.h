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

                auto nxt = _engine.get_targets(_map, loc, lane_model);
                assert(tests::test_eq(_engine, _planner));

                _planner.collision_avoidance();
                _planner.speed_control();
                _planner.build_path(t_, _planner.target_lane, _planner.target_speed, path, dt_);
            }

            return std::make_pair(path.x, path.y);
        }
    };
}