/*
 * visibility_map.cpp
 *
 * Graph Library for Autonomous and Dynamic Systems
 *
 * author:  Pierrick Koch <pierrick.koch@laas.fr>
 *          Cyril Robin <cyril.robin@laas.fr>
 * created: 2013-09-10
 * license: BSD
 */

#include "gladys/visibility_map.hpp"
#include "gladys/bresenham.hpp"

// Espilon, for float comparison
#ifndef EPS
#define EPS 0.05
#endif

namespace gladys {

void visibility_map::_load() {//{{{
    width  = dtm.get_width();
    height = dtm.get_height();
    // TODO cache stuff
}//}}}

/* computing function */
bool visibility_map::is_visible( const point_xy_t& s, const point_xy_t& t) const {//{{{
    // gdal::raster aka. vector<float>
    const auto& heightmap = get_heightmap();
    const auto& npointsmap = get_npointsmap();

    point_xyzt_t _s = rmdl.get_sensor_pose() ; // relative sensor position
    point_xy_t ns ;   // sensor position
    ns[0] = s[0] + _s[0] ; // x
    ns[1] = s[1] + _s[1] ; // y

    /* Check trivial cases : "s is next to t" or "t out of range" */
    double distance_st = distance( s, t );
    // TODO check if this threshold is valid (robot.radius)
    if ( distance_st < rmdl.get_radius()  + EPS )
        return true ;
    else if ( distance_st > rmdl.get_sensor_range() - EPS )
        return false ;

    /* Check if both s and t are known (we need zmax !)
     * else we cannot say if they are visible or not,
     * and assume there is no visibility link by default */
    if ( get_npointsmap()[ idx(s) ] < 1 - EPS
    ||   get_npointsmap()[ idx(t) ] < 1 - EPS)
        return false ;

    // From now, dist( ns, t) > 0
    /* Get the projection of the visibility line with Bresenham's line algorithm */
    points_t line = bresenham( s, t ) ;

    /* Test the visibility link along the line  :
     * for each point from the Bresenham's line, we check the height :
     * the point must be in the negative half-plane defined by the direct line
     * between the sensor and the target, otherwise it breaks the visibility
     * link.
     */
    // Get the (s-t) line equation: ax + by + c = 0
    // where x = distance to the sensor (projection)
    // and y = height of the point
    double zs, zt, d0, a, d, z;
    zs = _s[2] + heightmap[ idx(s) ] ;   // height of the sensor
    zt = heightmap[ idx(t) ] ;           // height of the target
    d0 = distance( s, t ) ;

    a = (zs - zt) / d0 ; // d > 0
    // b = 1 and c = -zs

    // test the condition for each point of the line
    for ( auto& p: line) {
        // if p is unknown (never observed), then assume we can see though it.
        if ( get_npointsmap()[ idx(p) ] < 1 - EPS )
            continue;
        d = distance( s, p ) ;
        z = heightmap[ idx(p) ] ;
        if ( a*d + z - zs > 0 + EPS )
            return false ;
    }

    return true ;
}//}}}


} // namespace gladys
