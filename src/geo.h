#pragma once

#include <vector>

namespace pp {
    // miles/hour to meters/second
    static constexpr double c_mph2mps = 0.44704;

    // Shortcut for infinity
    static constexpr double c_inf = std::numeric_limits<double>::infinity();

    // miles to meters
    static constexpr double c_mile2meter = 1609.34;

    struct fpoint {
        double s, d;
    };

    struct cpoint {
        double x, y;
    };

    struct path {
        std::vector<double> x, y;

        size_t size() const
        {
            return x.size();
        }
        void append(const pp::cpoint xy)
        {
            x.push_back(xy.x);
            y.push_back(xy.y);
        }

        void append(const double x_, const double y_)
        {
            x.push_back(x_);
            y.push_back(y_);
        }
    };

    
    inline constexpr double mph2mps(double x)
    {
        return x * c_mph2mps;
    }

    inline constexpr double mps2mph(double x)
    {
        return x / c_mph2mps;
    }

    inline constexpr double miles2meters(double x)
    {
        return x * c_mile2meter;
    }

    inline constexpr double meters2miles(double x)
    {
        return x / c_mile2meter;
    }

    inline double edot(double x1, double y1, double x2, double y2)
    {
        return x1 * x2 + y1 * y2;
    }

    inline double enorm(double x, double y)
    {
        return sqrt(x * x + y * y);
    }

    // euclidean distance
    inline double edistance(double x1, double y1, double x2, double y2)
    {
        return enorm(x2 - x1, y2 - y1);
    }
}