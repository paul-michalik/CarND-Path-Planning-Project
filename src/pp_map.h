#pragma once

#include "udacity.h"

namespace pp {
    class map { 
        std::vector<double> _map_waypoints_x;
        std::vector<double> _map_waypoints_y;
        std::vector<double> _map_waypoints_dx;
        std::vector<double> _map_waypoints_dy;
        std::vector<double> _map_waypoints_s;
    public:
        map(std::vector<double> map_waypoints_x_,
            std::vector<double> map_waypoints_y_,
            std::vector<double> map_waypoints_dx_,
            std::vector<double> map_waypoints_dy_,
            std::vector<double> map_waypoints_s_)
            : _map_waypoints_x{map_waypoints_x_}
            , _map_waypoints_y{map_waypoints_y_}
            , _map_waypoints_dx{map_waypoints_dx_}
            , _map_waypoints_dy{map_waypoints_dy_}
            , _map_waypoints_s{map_waypoints_s_}
        {}

        auto get_frenet(double x_, double y_, double theta_) const
        {
            return ::udacity::getFrenet(x_, y_, theta_, _map_waypoints_x, _map_waypoints_y);
        }

        auto get_xy(double s_, double d_) const
        {
            return ::udacity::getXY(s_, d_, _map_waypoints_s, _map_waypoints_x, _map_waypoints_y);
        }
    };
}