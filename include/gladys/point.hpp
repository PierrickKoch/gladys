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
#include <deque>
#include <ostream>
#include <sstream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace gladys {

typedef std::array<double, 2> point_xy_t;  // XY
typedef std::array<double, 3> point_xyz_t; // XYZ
typedef std::array<double, 4> point_xyzt_t; // XYZ + Theta
typedef std::vector<point_xy_t> points_t; // list of points
typedef std::deque<point_xy_t> path_t; // path = deque for push_front

template <typename T>
std::string to_string(const T& t)
{
    std::ostringstream oss;
    oss << t;
    return oss.str();
}

template<typename Container>
inline std::ostream& stream_it(std::ostream& os, Container& c)
{
    bool first = true;
    os << "[";
    for (auto& v : c) {
        if (first)
            first = false;
        else
            os << ", ";
        os << v;
    }
    return os << "]";
}

template <typename T>
inline std::ostream& operator<<(std::ostream& os, const std::vector<T>& v) {
    return stream_it(os, v);
}

template <typename T>
inline std::ostream& operator<<(std::ostream& os, const std::deque<T>& v) {
    return stream_it(os, v);
}

template <typename T, size_t N>
inline std::ostream& operator<<(std::ostream& os, const std::array<T, N>& v) {
    return stream_it(os, v);
}

inline bool operator> (const points_t &v1, const points_t& v2) {
    return (v1.size() > v2.size() );
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
inline double distance_sq(const point_xyzt_t& pA, const point_xyzt_t& pB) {
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

/* angle computation */
// compute yaw ; return value in ]-Pi,Pi]
// this version should be use when y-axis is inverted (goes down)
inline double yaw_angle_y_inv(const point_xy_t& pA, const point_xy_t& pB) {
    double yaw = std::atan2(pB[1] - pA[1], - pB[0] + pA[0]) ; // use (-y)
    yaw = std::fmod(yaw, (2*M_PI)); //modulo
    if ( yaw >   M_PI ) yaw -= 2*M_PI; // consider yaw in ]-Pi,Pi]
    if ( yaw <= -M_PI ) yaw += 2*M_PI; // consider yaw in ]-Pi,Pi]

    return yaw;
}

} // namespace gladys

#endif // POINT_HPP

