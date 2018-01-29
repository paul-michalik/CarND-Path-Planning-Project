#pragma once

#include <cstddef>
#include <limits>

namespace pp {
    // miles/hour to meters/second
    static constexpr double c_mph2mps = 0.44704;

    // Shortcut for infinity
    static constexpr double c_inf = std::numeric_limits<double>::infinity();

    // miles to meters
    static constexpr double c_mile2meter = 1609.34;

    // paths are always 50 points long
    constexpr std::size_t c_path_point_count = 50u;

    // Speed limit in mph
    constexpr double c_speed_limit_mph = 50; // [mph]

    // Target acceleration
    constexpr double c_acceleration = 0.16; // [m/s^2]

    // Observable horizon in the lane environment model
    constexpr double c_lane_min_horizon = 50; // [m]

    // Safe distance buffer for lane changes in front of me
    constexpr double c_lane_change_min_front_buffer = 15; // [m]

    // Safe distance buffer for lane changes at the back
    constexpr double c_lane_change_min_back_buffer = 7; // [m]

    // Number of lanes in this driving direction
    constexpr int    c_lane_count = 3;

    // Lane width in m
    constexpr double c_lane_width = 4; // [m]

    // Safe lane distance from boundary
    constexpr double c_lane_dist_safety_margin = 0.05; // [m]
}