#pragma once

#include "geo.h"
#include <vector>
#include <json.hpp>

namespace pp {
    // Sensor Fusion data entry
    struct sensor_data {
        // Unique identifier of a car
        int uid;

        // Position in map coordinates
        double x, y;

        // Velocity in map coordinates direction (m/s)
        double vx, vy;

        // Position in Frenet coordinates
        double s, d;
    };

    // Main car's localization data
    struct ego_position {
        // ego position in map coordinates
        double x, y;

        // ego yaw angle in map coordinates (radians)
        double yaw;

        // ego speed (m/s)
        double speed;

        // position in Frenet frame coordinates
        double s, d;
    };

    struct telemetry_data {
        ego_position ego;
                      
        // Previous path data given to the Planner
        pp::path previous_path;

        // Previous path's end s and d values
        pp::fpoint end_path;

        // Sensor Fusion Data, a list of all other cars on the same side of the road.
        std::vector<sensor_data> sensor_fusion;
    };

    // Convert from JSON to sensor data
    inline void from_json(const nlohmann::json & j_, sensor_data &s_)
    {
        s_.uid = j_[0].get<int>();
        s_.x = j_[1];
        s_.y = j_[2];
        s_.vx = j_[3];
        s_.vy = j_[4];
        s_.s = j_[5];
        s_.d = j_[6];
    }

    // Convert from JSON to telemetry data
    inline void from_json(nlohmann::json const& j_, telemetry_data &t_)
    {
        t_.ego.x = j_["x"];
        t_.ego.y = j_["y"];
        t_.ego.yaw = udacity::deg2rad(j_["yaw"]);
        t_.ego.s = j_["s"];
        t_.ego.d = j_["d"];
        t_.ego.speed = pp::mph2mps(j_["speed"]);

        // Previous path data given to the Planner
        t_.previous_path.x = j_["previous_path_x"].get<std::vector<double>>();
        t_.previous_path.y = j_["previous_path_y"].get<std::vector<double>>();
        // Previous path's end s and d values
        t_.end_path.s = j_["end_path_s"];
        t_.end_path.d = j_["end_path_d"];

        // Sensor Fusion Data, a list of all other cars on the same side of the road.
        t_.sensor_fusion = j_["sensor_fusion"].get<std::vector<sensor_data>>();
    }
}