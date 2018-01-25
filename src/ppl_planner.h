#pragma once

namespace pp_l {
    using namespace std;
   
    // waypoint
    struct waypoint_t {
        double x, y, s, dx, dy;
    };
    // waypoint list as a set of vectors
    struct waypoints_list {
        vector<double> x, y, s, dx, dy;
        size_t size() const
        {
            return x.size();
        }
        waypoint_t operator[](int i) const
        {
            return {x[i],y[i],s[i],dx[i],dy[i]};
        }
    };

    // Helper representing the set of lanes in the highway.
    // The origin is in the middle of the road; from there lanes
    // are numbered: 0, 1, 2, ...
    struct Track
    {
        // These are actually constants
        double speed_limit_mph = 50;
        int    lane_count = 3;
        double lane_width = 4;

        // Track width
        double width() const
        {
            return lane_count * lane_width;
        }

        // Lane center from the origin
        double lane_center(int lane) const
        {
            return (0.5 + lane) * lane_width;
        }

        // Lane center with a bias in the left and right lanes
        double safe_lane_center(int lane) const
        {
            double center_ = lane_center(lane);
            if (lane == 0) // left lane
                center_ += 0.05;
            else if (lane == lane_count - 1) // right lane
                center_ -= 0.05;
            return center_;
        }

        // Lane for a given frenet point
        int lane_at(pp::fpoint const & fp) const
        {
            return lane_at(fp.d);
        }

        // Lane at a given distance from the road center
        int lane_at(double d) const
        {
            if (d < 0)
                return -1;
            if (d > width())
                return -2;
            return std::floor(d / lane_width);
        }

        double distance_to_lane(double d, int lane)
        {
            return lane_center(lane) - d;
        }
    };

    struct lane_info_t
    {
        int    front_car = -1;
        int    back_car = -1;
        double front_gap = pp::c_inf;
        double front_speed = pp::c_inf;
        double back_gap = pp::c_inf;
        double back_speed = 0;
        bool   feasible = true;

        bool is_clear() const
        {
            return feasible && front_car < 0;
        }
    };

    // Path Planner
    struct PathPlanner {
        pp::map   roadmap;
        Track     track;

        double    accel = 0.2;     // m/s^2
        double    lane_horizon = 50;   // m
        double    lane_change_front_buffer = 15; // m
        double    lane_change_back_buffer = 7;  // m

                                                // number of points to generate ... constant
        int       n_path_points = 50;

        // Reference position for new path - 2 points to build a tangent
        double    ref_x;
        double    ref_y;
        double    ref_x_prev;
        double    ref_y_prev;
        double    ref_yaw;
        double    ref_speed;
        double    ref_s;
        double    ref_d;
        int       ref_lane;
        int       ref_points;  // number of steps consumed at reference 

        vector<lane_info_t> lane_info;

        // lap tracking for the ego
        size_t    ego_laps;
        size_t    ego_laps_tick;
        pp::fpoint      ego_start_position;
        bool      ego_passed_zero_s;

        // target lane for changing lane states
        int       changing_lane = -1;
        // target lane for next path (of left=0, middle=1, right=2)
        int       target_lane = 1;
        // target speed for next path
        double    target_speed = 0;

        enum class STATE {
            START, KL, PLC, LC
        };
        STATE     state_ = STATE::KL;
        double    state_s_;

    public:
        void initialize(const pp::map & map_)
        {
            roadmap = map_;
        }

        // Reset planner
        void reset()
        {
            target_lane = 1;
            target_speed = 0;
            ref_points = 0;
            ego_laps = 0;
            ego_laps_tick = 0;
            ego_passed_zero_s = false;
            state_ = STATE::START;
            state_s_ = 0;
        }

        // Run the planner with the given telemetry to generate the next trajectory.
        // dt is the simulator period.
        void run(const pp::telemetry_data & ego, pp::path & path, double dt);

    public:

        void track_lap(const pp::telemetry_data &ego);
        void compute_reference(const pp::telemetry_data & ego, double dt);
        void process_sensor_fusion(const pp::telemetry_data & ego, double dt);
        void create_plan(const pp::telemetry_data & ego, double dt);
        void collision_avoidance();
        void speed_control();
        void build_path(const pp::telemetry_data & ego,
            const int target_lane,
            const double target_speed,
            pp::path & path,
            double dt);

        int get_best_lane() const;
        void set_state(const pp::telemetry_data & ego, STATE new_state);
    };

    // == Implementations =================================

    void PathPlanner::run(const pp::telemetry_data & ego, pp::path & path, double dt)
    {
        //std::cout << __FUNCTION__ << std::endl;

        // 1a. Get reference point: where we start to build the plan
        compute_reference(ego, dt);
        // 1b. Track laps just for the shake of it
        track_lap(ego);
        // 2. Analyse the environment
        process_sensor_fusion(ego, dt);
        // 3. Behaviour; set target lane and speed
        create_plan(ego, dt);
        // 4. Avoid collisions
        collision_avoidance();
        // 5. Control speed
        speed_control();
        // 6. Generate final trajectory
        build_path(ego, target_lane, target_speed, path, dt);
    }

    void PathPlanner::track_lap(const pp::telemetry_data & t_)
    {
        //std::cout << __FUNCTION__ << std::endl;

        if (ego_laps_tick == 0) {
            ego_start_position = {t_.ego.s, t_.ego.d};
        }

        // EGO .. current position
        if (t_.ego.s < ego_start_position.s) {
            ego_passed_zero_s = true;
        }

        if (ego_passed_zero_s && t_.ego.s > ego_start_position.s) {
            ego_laps++;
            ego_laps_tick = 0;
            ego_passed_zero_s = false;

            //std::cout << "#################################################################";
        } else {
            //std::cout << "_________________________________________________________________";
        }

        ego_laps_tick++;

        /*std::cout
        << endl
        << endl
        << "LAP " << (ego_laps + 1)
        << "\tLANE " << track.lane_at(ego.d)
        << " (s=" << fixed << setprecision(1) << ego.s
        << ", d=" << fixed << setprecision(1) << ego.d << ")"
        << " PLANNED " << ego.previous_path.size() << " points"
        << endl;*/
    }

    void PathPlanner::compute_reference(const pp::telemetry_data & t_, double dt)
    {
        //std::cout << __FUNCTION__ << std::endl;

        const int planned_size = t_.previous_path.size();

        // If previous path almost empty, use the current ego position
        if (planned_size < 2)
        {
            ref_x = t_.ego.x;
            ref_y = t_.ego.y;
            ref_yaw = t_.ego.yaw;
            ref_speed = t_.ego.speed;
            ref_s = t_.ego.s;
            ref_d = t_.ego.d;

            ref_x_prev = ref_x - cos(ref_yaw);
            ref_y_prev = ref_y - sin(ref_yaw);
        } else // use the previous path
        {
            ref_x = *(t_.previous_path.x.end() - 1);
            ref_y = *(t_.previous_path.y.end() - 1);

            ref_x_prev = *(t_.previous_path.x.end() - 2);
            ref_y_prev = *(t_.previous_path.y.end() - 2);

            ref_yaw = std::atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
            ref_speed = pp::edistance(ref_x_prev, ref_y_prev, ref_x, ref_y) / dt;

            ref_s = t_.end_path.s;
            ref_d = t_.end_path.d;
        }

        ref_lane = track.lane_at(ref_d);

        // FIXME assuming n_path_points keeps constant. If not, we have to
        // track the size of the previous path.
        ref_points += n_path_points - planned_size;
    }

    void PathPlanner::process_sensor_fusion(const pp::telemetry_data & t_, double dt)
    {
        //std::cout << __FUNCTION__ << std::endl;

        lane_info.clear();
        lane_info.resize(track.lane_count);

        //std::cout << "SENSOR FUSION" << endl;

        const int planned_size = t_.previous_path.size();

        for (auto & car : t_.sensor_fusion) {
            int car_lane = track.lane_at(car.d);

            // vehicle in the road
            if (car_lane >= 0) {
                auto & lane = lane_info[car_lane];

                // predict car position assuming constant speed
                auto car_speed = pp::enorm(car.vx, car.vy);
                // predict car position after consumption of pending plan
                auto car_next_s = car.s + car_speed * planned_size * dt;
                // check if it's in front or behind
                bool in_front = car_next_s > ref_s;
                // absolute s-distance
                auto car_gap = fabs(car_next_s - ref_s);

                /*std::cout << "  CAR " << setw(2) << car.uid
                << "  lane=" << car_lane
                << "  v=" << setw(4) <<pp::mps2mph(norm(car.vx, car.vy))
                << "  s=" << setw(6) << car.s
                << "  s'=" << setw(6) << car_next_s
                << "  gap=" << setw(7) << (car_next_s - ref_s)
                << endl;*/

                // if getting under the buffer
                if (in_front && car_gap < lane.front_gap) {
                    lane.front_car = car.uid;
                    lane.front_gap = car_gap;
                    lane.front_speed = car_speed;
                } else if (car_gap < fmin(lane.back_gap, lane_horizon)) {
                    lane.back_car = car.uid;
                    lane.back_gap = car_gap;
                    lane.back_speed = car_speed;
                }

                // Evaluate feasibility
                lane.feasible = (lane.front_gap > lane_change_front_buffer)
                    && (lane.back_gap > lane_change_back_buffer);
            }
        }

        //#ifndef NDEBUG
        //        {
        //            auto & log_ = std::cout;
        //            log_ << "LANE INFO" << endl;
        //            for (int i = 0; i < lane_info.size(); i++) {
        //                auto & l = lane_info[i];
        //                log_ << "  LANE " << i << ": ";
        //                if (l.is_clear())
        //                    log_ << "CLEAR";
        //                else {
        //                    log_ << (l.feasible ? "FEASIBLE " : "         ");
        //                    log_ << "[";
        //                    log_ << fixed << setw(2) << setprecision(0);
        //                    log_ << l.front_gap;
        //                    if (isfinite(l.front_gap))
        //                        log_ << "+" <<pp::mps2mph(l.front_speed) << "t";
        //                    log_ << "; ";
        //                    log_ << -l.back_gap;
        //                    if (isfinite(l.back_gap))
        //                        log_ << "+" <<pp::mps2mph(l.back_speed) << "t";
        //                    log_ << "]";
        //                }
        //                log_ << endl;
        //            }
        //        }
        // #endif
    }

    void PathPlanner::set_state(const pp::telemetry_data & t_, STATE new_state)
    {
        //std::cout << __FUNCTION__ << std::endl;

        if (state_ != new_state) {
            // Set the new state and the reference position
            state_ = new_state;
            state_s_ = ref_s;
        }
    }

    auto pp_get_lane_speed(PathPlanner const& p_, lane_info_t l_)
    {
        return l_.front_gap > p_.lane_horizon
            ? pp::c_inf
            : l_.back_gap < p_.lane_change_back_buffer
            ? l_.back_speed
            : l_.front_speed;
    }

    auto pp_get_best_lane(PathPlanner const& p_)
    {
        using best_lane_info_t = std::tuple<int, double, lane_info_t>;
        std::vector<best_lane_info_t> t_lane_info;
        {
            int t_counter = 0;
            std::transform(
                p_.lane_info.begin(), p_.lane_info.end(),
                std::back_inserter(t_lane_info),
                [&](auto const& l_) {
                return std::make_tuple(
                    t_counter++,
                    pp_get_lane_speed(p_, l_),
                    l_);
            });
        }

        // pick only clear lanes <2> with infinite velocities <1>
        t_lane_info.erase(std::remove_if(t_lane_info.begin(), t_lane_info.end(), [&](best_lane_info_t const& lane_) {
            return !get<2>(lane_).is_clear() && get<1>(lane_) < pp::c_inf;
        }), t_lane_info.end());

        // sort lanes by distance to reference lane
        std::sort(t_lane_info.begin(), t_lane_info.end(), [&](auto const& l_, auto const& r_) {
            return abs(get<0>(l_) - p_.ref_lane) < abs(get<0>(l_) - p_.ref_lane);
        });

        return !t_lane_info.empty() ? get<0>(t_lane_info.front()) : p_.ref_lane;
    }

    int PathPlanner::get_best_lane() const
    {
        //std::cout << __FUNCTION__ << std::endl;
#ifdef PP_GET_BEST_LANE
        return pp_get_best_lane(*this);
#else
        if (lane_info[1].is_clear())
            return 1;

        vector<int> lanes(track.lane_count);
        iota(lanes.begin(), lanes.end(), 0);

        std::sort(lanes.begin(), lanes.end(), [&](int i, int j) {
            auto & lane_i = lane_info[i];
            auto & lane_j = lane_info[j];

            auto i_closest_to_ref_lane = abs(i - ref_lane) <= abs(j - ref_lane);

            // prefer clear lanes
            if (lane_i.is_clear()) {
                if (lane_j.is_clear())
                    return i_closest_to_ref_lane;
                else
                    return true;
            } else if (lane_j.is_clear()) {
                return false;
            }


            const double i_v = lane_i.front_gap > lane_horizon
                ? pp::c_inf
                : lane_i.back_gap < lane_change_back_buffer
                ? lane_i.back_speed
                : lane_i.front_speed;
            const double j_v = lane_j.front_gap > lane_horizon
                ? pp::c_inf
                : lane_j.back_gap < lane_change_back_buffer
                ? lane_j.back_speed
                : lane_j.front_speed;

            if ((!isfinite(i_v) && !isfinite(j_v)) || fabs(i_v - j_v) < 0.5)
                return lane_i.front_gap >= lane_j.front_gap;
            else
                return i_v > j_v;
        });

        return lanes[0];
#endif
    }

    void PathPlanner::create_plan(const pp::telemetry_data & t_, double dt)
    {
        //std::cout << __FUNCTION__ << std::endl;

        //std::cout << "PLANNING" << endl;

        // Give it a safety margin
        const double road_speed_limit = pp::mph2mps(track.speed_limit_mph) - 0.2;
        const double cte = (ref_d - track.lane_center(target_lane));

        if (state_ == STATE::START)
        {
            changing_lane = -1;
            target_lane = ref_lane;
            state_ = STATE::KL;
            state_s_ = ego_start_position.s;
        }

        int best_lane = get_best_lane();
        std::cout << " * BEST LANE " << best_lane << endl;

        while (true)
        {
            double meters_in_state = ref_s - state_s_;
            while (meters_in_state < 0)
                meters_in_state += roadmap.get_max_s();

            if (state_ == STATE::KL)
            {
                assert(target_lane == ref_lane);

                std::cout << " * KEEPING LANE " << target_lane
                    << " FOR " << setprecision(2) << pp::meters2miles(meters_in_state) << " Miles"
                    << " (cte=" << setprecision(1) << setw(4) << cte << " m)"
                    << endl;

                target_speed = road_speed_limit;

                // Evaluate lane change
                changing_lane = -1;

                // Evaluate lane changes if current lane is busy
                if (lane_info[target_lane].front_gap < lane_horizon
                    && lane_info[target_lane].front_speed < road_speed_limit)
                {
                    // Change if we are not in the best one
                    if (lane_info[best_lane].front_speed > lane_info[target_lane].front_speed + 0.2) {
                        changing_lane = best_lane;
                        set_state(t_, STATE::PLC);
                        continue;
                    }
                }

                break;
            }

            else if (state_ == STATE::PLC)
            {
                assert(changing_lane != ref_lane);

                std::cout << " * PREPARING CHANGE TO LANE " << changing_lane
                    << " FOR " << setprecision(2) <<pp::meters2miles(meters_in_state) << " Miles"
                    << " (cte=" << setprecision(1) << setw(4) << cte << " m)"
                    << endl;

                target_lane = ref_lane + ((changing_lane > ref_lane) ? 1 : -1);

                if (lane_info[target_lane].feasible)
                {
                    set_state(t_, STATE::LC);
                    continue;
                } else if (changing_lane != best_lane || meters_in_state > 500)
                {
                    // Waiting too much or not the best: cancel
                    target_lane = ref_lane;
                    set_state(t_, STATE::KL);
                    continue;
                } else // Not feasible
                {
                    if (lane_info[ref_lane].front_gap < 30) {
                        // Can't perform the change ... try to slow down here
                        if (lane_info[target_lane].back_gap < lane_change_back_buffer)
                            target_speed = fmin(target_speed, lane_info[target_lane].back_speed - 0.5);
                        else
                            target_speed = fmin(target_speed, lane_info[target_lane].front_speed - 0.5);
                    }
                    // But wait in this lane
                    target_lane = ref_lane;
                }

                break;
            }

            else if (state_ == STATE::LC)
            {
                if (ref_lane == target_lane && fabs(cte) < 0.2 && meters_in_state > 100)
                {
                    // Lane change completed
                    if (changing_lane >= 0 && changing_lane != ref_lane) {
                        set_state(t_, STATE::PLC);
                        continue;
                    }

                    changing_lane = -1;
                    set_state(t_, STATE::KL);
                    continue;
                }

                if (fmin(lane_info[target_lane].front_gap, lane_info[target_lane].back_gap) < 5)
                {
                    // ABORT
                    target_lane = ref_lane;
                    changing_lane = -1;
                    std::cout << " * ABORTING LANE CHANGE" << endl;
                }

                std::cout << " * CHANGING TO LANE " << target_lane
                    << " FOR " << setprecision(2) <<pp::meters2miles(meters_in_state) << " Miles"
                    << " (error=" << setprecision(1) << setw(4) << cte << " m)"
                    << endl;

                target_speed = road_speed_limit;
                break;
            }

            break;
        }

        // Ensure the target speed is inside the limits
        // NOTE that we don't consider the possibility of moving backwards.
        target_speed = fmax(0.0, fmin(road_speed_limit, target_speed));
    }

    void PathPlanner::collision_avoidance()
    {
        //std::cout << __FUNCTION__ << std::endl;

        // Avoid collisions
        if (lane_info[target_lane].front_gap < 30)
        {
            if (lane_info[target_lane].front_gap < 15)
                target_speed = fmax(0, fmin(target_speed, lane_info[target_lane].front_speed - 0.2));
            else
                target_speed = fmin(target_speed, lane_info[target_lane].front_speed);

            std::cout << " * FOLLOWING THE LEAD (" << lane_info[target_lane].front_gap << " m)" << endl;
        }
    }

    void PathPlanner::speed_control()
    {
        //std::cout << __FUNCTION__ << std::endl;

        // Adjust speed
        if (target_speed < ref_speed) {
            // decelerate
            target_speed = fmax(target_speed, ref_speed - accel);
        } else if (target_speed > ref_speed) {
            // accelerate
            target_speed = fmin(target_speed, ref_speed + accel);
        }
    }


    void PathPlanner::build_path(const pp::telemetry_data & t_,
        const int target_lane,
        const double target_speed,
        pp::path & path, double dt)
    {
        //std::cout << __FUNCTION__ << std::endl;

        std::cout << "TRAJECTORY" << endl
            << " * TARGET LANE " << target_lane << endl
            << " * TARGET SPEED " << setprecision(1) <<pp::mps2mph(target_speed)
            << endl;

        const auto target_d = track.safe_lane_center(target_lane);

        // trajectory points
        pp::path anchors;

        // Build a path tangent to the previous end state
        anchors.append(ref_x_prev, ref_y_prev);
        anchors.append(ref_x, ref_y);

        // Add 3 more points spaced 30 m
        for (int i = 1; i <= 3; i++) {
            auto next_wp = roadmap.get_xy(ref_s + 30 * i, target_d);
            anchors.append(next_wp);
        }

        // change the points to the reference (ref_) coordinate
        for (int i = 0; i < anchors.size(); i++) {
            const auto dx = anchors.x[i] - ref_x;
            const auto dy = anchors.y[i] - ref_y;
            anchors.x[i] = dx * cos(-ref_yaw) - dy * sin(-ref_yaw);
            anchors.y[i] = dx * sin(-ref_yaw) + dy * cos(-ref_yaw);
        }

        // Interpolate the anchors with a cubic spline
        tk::spline spline;
        spline.set_points(anchors.x, anchors.y);

        // Now we can build the final trajectory...

        // Add previous path for continuity
        path.x.assign(t_.previous_path.x.begin(), t_.previous_path.x.end());
        path.y.assign(t_.previous_path.y.begin(), t_.previous_path.y.end());

        // distance = N * dt * speed

        // set a horizon of 30 m
        const double target_x = 30;
        const double target_y = spline(target_x);
        const double target_dist = pp::enorm(target_x, target_y);

        // t = N * dt = target_dist / target_speed
        // const double N = target_dist / (dt * target_speed);
        const double t = target_x / target_dist * dt;

        // sample the spline curve to reach the target speed
        for (int i = 1; i <= n_path_points - path.x.size(); i++) {
            double x__ = i * t * target_speed;
            double y__ = spline(x__);
            // transform back to world coordinates
            double x_ = x__ * cos(ref_yaw) - y__ * sin(ref_yaw) + ref_x;
            double y_ = x__ * sin(ref_yaw) + y__ * cos(ref_yaw) + ref_y;
            // append the trajectory points
            path.append({x_, y_});
        }
    }
}