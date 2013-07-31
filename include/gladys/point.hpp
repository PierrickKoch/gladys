/*
 * point.hpp
 *
 * Graph Library for Autonomous and Dynamic Systems
 *
 * author:  Pierrick Koch <pierrick.koch@laas.fr>
 * created: 2013-07-22
 * license: BSD
 */
#ifndef POINT_HPP
#define POINT_HPP

#include <cmath>
#include <array>
#include <vector>
#include <ostream>
#include <functional>

namespace gladys {

typedef std::array<double, 2> point_xy_t;  // XY
typedef std::array<double, 3> point_xyz_t; // XYZ
typedef std::vector<point_xy_t> points_t; // list of points
typedef std::deque<point_xy_t> path_t; // path = deque for push_front

typedef std::function<void(const point_xy_t&, int)> display_hook_t;

inline std::string to_string(const point_xy_t& value ) {
    return "[" + std::to_string(value[0]) + "," +
                 std::to_string(value[1]) + "]";
}
inline std::ostream& operator<<(std::ostream& os, const point_xy_t& value) {
    return os<<to_string(value);
}

inline std::string to_string(const point_xyz_t& value ) {
    return "[" + std::to_string(value[0]) + "," +
                 std::to_string(value[1]) + "," +
                 std::to_string(value[2]) + "]";
}
inline std::ostream& operator<<(std::ostream& os, const point_xyz_t& value) {
    return os<<to_string(value);
}

inline std::string to_string(const path_t& value ) {
    std::string arrow = "", buff = "";
    for (auto& elt : value) {
        buff += arrow + to_string(elt);
        arrow = " -> ";
    }
    return buff;
}
inline std::ostream& operator<<(std::ostream& os, const path_t& value) {
    return os<<to_string(value);
}


/** Euclidian distance (squared)
 * usefull to compare a set of points (faster)
 */
inline double distance_sq(const point_xy_t& pA, const point_xy_t& pB) {
    double x = pA[0] - pB[0];
    double y = pA[1] - pB[1];
    return x*x + y*y;
}
inline double distance_sq(const point_xyz_t& pA, const point_xyz_t& pB) {
    double x = pA[0] - pB[0];
    double y = pA[1] - pB[1];
    double z = pA[2] - pB[2];
    return x*x + y*y + z*z;
}
/** Euclidian distance */
template <class Point>
inline double distance(const Point& pA, const Point& pB) {
    return std::sqrt(distance_sq(pA, pB));
}

} // namespace gladys

#endif // POINT_HPP

