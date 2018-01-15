#pragma once

#include "pp_map.h"
#include <utility>

namespace pp {
    class planner {
        pp::map _map;
    public:
        planner(map map_)
            : _map(map_)
        {}

        auto get_next_vals() const
        {
            return std::make_pair(std::vector<double>{}, std::vector<double>{});
        }
    };
}