#pragma once

#include "udacity.h"
#include "pp_geo.h"

namespace pp_l {
    class RoadMap;
}
namespace pp {
    class map { 
        std::vector<double> _map_waypoints_x;
        std::vector<double> _map_waypoints_y;
        std::vector<double> _map_waypoints_s;
        std::vector<double> _map_waypoints_dx;
        std::vector<double> _map_waypoints_dy;
        std::string _filename;
    public:
        friend pp_l::RoadMap;

        map() = default;

        map(std::vector<double> map_waypoints_x_,
            std::vector<double> map_waypoints_y_,
            std::vector<double> map_waypoints_s_,
            std::vector<double> map_waypoints_dx_,
            std::vector<double> map_waypoints_dy_,
            std::string filename_)
            : _map_waypoints_x{map_waypoints_x_}
            , _map_waypoints_y{map_waypoints_y_}
            , _map_waypoints_s{map_waypoints_s_}
            , _map_waypoints_dx{map_waypoints_dx_}
            , _map_waypoints_dy{map_waypoints_dy_}
            , _filename{filename_}
        {}

        auto get_max_s() const
        {
            return udacity::c_max_s;
        }

        auto get_frenet(double x_, double y_, double theta_) const
        {
            auto const& f = udacity::getFrenet(x_, y_, theta_, _map_waypoints_x, _map_waypoints_y);
            return pp::fpoint{f[0], f[1]};
        }

        auto get_xy(double s_, double d_) const
        {
            auto const& e = ::udacity::getXY(s_, d_, _map_waypoints_s, _map_waypoints_x, _map_waypoints_y);
            return pp::cpoint{e[0], e[1]};
        }

        auto get_filename() const
        {
            return _filename;
        }
    };
}