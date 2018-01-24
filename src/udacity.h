#pragma once

#include <vector>

constexpr double pi();
double deg2rad(double x);
double rad2deg(double x);
double distance(double x1, double y1, double x2, double y2);
int ClosestWaypoint(double x, double y, const std::vector<double> &maps_x, const std::vector<double> &maps_y);
int NextWaypoint(double x, double y, double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y);

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
std::vector<double> getFrenet(double x, double y, double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y);

// Transform from Frenet s,d coordinates to Cartesian x,y
std::vector<double> getXY(double s, double d, const std::vector<double> &maps_s, const std::vector<double> &maps_x, const std::vector<double> &maps_y);

extern const double c_max_s;
namespace udacity {
    using ::pi;
    using ::deg2rad;
    using ::rad2deg;
    using ::distance;
    using ::ClosestWaypoint;
    using ::NextWaypoint;
    using ::getFrenet;
    using ::getXY;
    using ::c_max_s;
}