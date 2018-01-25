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
    struct telemetry_data {
        // ego position in map coordinates
        double x, y;

        // ego yaw angle in map coordinates (radians)
        double yaw; 

        // ego speed (m/s)
        double speed; 

        // position in Frenet frame coordinates
        double s, d;     
                      
        // Previous path data given to the Planner
        pp::path previous_path;

        // Previous path's end s and d values
        pp::fpoint end_path;

        // Sensor Fusion Data, a list of all other cars on the same side of the road.
        std::vector<sensor_data> sensor_fusion;
    };

    // Convert from JSON to sensor data
    void from_json(const nlohmann::json & j, sensor_data &sensor)
    {
        sensor.uid = j[0].get<int>();
        sensor.x = j[1];
        sensor.y = j[2];
        sensor.vx = j[3];
        sensor.vy = j[4];
        sensor.s = j[5];
        sensor.d = j[6];
    }

    // Convert from JSON to telemetry data
    void from_json(const nlohmann::json & j, telemetry_data &tele)
    {
        tele.x = j["x"];
        tele.y = j["y"];
        tele.yaw = deg2rad(j["yaw"]);
        tele.s = j["s"];
        tele.d = j["d"];
        tele.speed = mph2mps(j["speed"]);

        // Previous path data given to the Planner
        tele.previous_path.x = j["previous_path_x"].get<std::vector<double>>();
        tele.previous_path.y = j["previous_path_y"].get<std::vector<double>>();
        // Previous path's end s and d values
        tele.end_path.s = j["end_path_s"];
        tele.end_path.d = j["end_path_d"];

        // Sensor Fusion Data, a list of all other cars on the same side of the road.
        tele.sensor_fusion = j["sensor_fusion"].get<std::vector<sensor_data>>();
    }
}