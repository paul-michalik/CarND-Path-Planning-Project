#pragma once

#include <vector>
#include <spline.h>

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

        void append(const pp::cpoint p_)
        {
            x.push_back(p_.x);
            y.push_back(p_.y);
        }

        void append(const double x_, const double y_)
        {
            x.push_back(x_);
            y.push_back(y_);
        }
    };

    // interpolated curve
    class spline_curve {
        tk::spline _sx;
        tk::spline _sy;
    public:
        void fit(
            std::vector<double> const& s_,
            std::vector<double> const& x_, 
            std::vector<double> const& y_)
        {
            _sx.set_points(s_, x_);
            _sy.set_points(s_, y_);
        }

        pp::cpoint get(double s_) const
        {
            return {_sx(s_), _sy(s_)};
        }

        double get_x(double s_) const
        {
            return _sx(s_);
        }

        double get_y(double s_) const
        {
            return _sy(s_);
        } 
    };

    constexpr double mph2mps(double x)
    {
        return x * c_mph2mps;
    }

    constexpr double mps2mph(double x)
    {
        return x / c_mph2mps;
    }

    constexpr double miles2meters(double x)
    {
        return x * c_mile2meter;
    }

    constexpr double meters2miles(double x)
    {
        return x / c_mile2meter;
    }

    inline double edot(double x1, double y1, double x2, double y2)
    {
        return x1 * x2 + y1 * y2;
    }

    // euclidian norm
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