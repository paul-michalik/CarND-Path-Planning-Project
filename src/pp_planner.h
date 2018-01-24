#pragma once

#include "pp_map.h"
#include "pp_geo.h"
#include "udacity.h"
#include <spline.h>
#include <json.hpp>
#include <utility>
#include <limits>
#include <algorithm>
#include <vector>
#include <tuple>
#include "ppl_planner.h"

namespace pp {
    class planner {
        pp::map _map;

        mutable pp_l::PathPlanner _planner;

    public:
        planner(pp::map map_)
            : _map(map_)
        {
            //_planner.initialize(_map.get_filename());
            _planner.initialize(_map);
            _planner.reset();
        }

        auto get_next_vals(pp_l::telemetry_data const& env_, double dt_) const
        {
            //std::cout
            //    << __FUNCTION__ << std::endl
            //    << "Time difference: " << dt_ << std::endl;

            pp_l::path_t path;
            _planner.run(env_, path, dt_);

            return std::make_pair(path.x, path.y);
        }
    };
}