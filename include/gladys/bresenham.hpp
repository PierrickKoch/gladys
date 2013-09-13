/*
 * bresenham.hpp
 *
 * Graph Library for Autonomous and Dynamic Systems
 *
 * author:  Cyril Robin <cyril.robin@laas.fr>
 * created: 2013-09-13
 * license: BSD
 */

#ifndef BRESENHAM_HPP
#define BRESENHAM_HPP

#include "gladys/point.hpp"

namespace gladys {

/** Compute the Bresenham's line between s and t
 *
 * The implementation keeps the order of the points in the line (from s to t)
 *
 * @param s the starting position
 *
 * @param t the targetted position
 *
 * @returns the Bresenham's line.
 *
 */
points_t bresenham( const point_xy_t& s, const point_xy_t& t) ;


} // namespace gladys

#endif // BRESENHAM_HPP


