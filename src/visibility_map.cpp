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

bool visibility_map::is_visible( const point_xy_t& s, const point_xy_t& t) const {

    point_xyz_t s3D = {s[0], s[1], 0};
    point_xyz_t t3D = {t[0], t[1], 0};

    return is_visible(s3D, t3D);
}

bool visibility_map::is_sensor_visible( const point_xy_t& s, const point_xy_t& t) const {
    point_xyzt_t _s = rmdl.get_sensor_pose() ; // relative sensor position

    point_xyz_t s3D = {s[0] + _s[0], s[1] + _s[1], _s[2]};
    point_xyz_t t3D = {t[0], t[1], 0};

    double distance_st = distance( s3D, t3D );
    if ( distance_st > rmdl.get_sensor_range() - EPS )
        return false ;

    return is_visible(s3D, t3D);
}

bool visibility_map::is_sensor_visible( const point_xyz_t& s, const point_xyz_t& t) const {
    point_xyzt_t _s = rmdl.get_sensor_pose() ; // relative sensor position

    point_xyz_t s3D = {s[0] + _s[0], s[1] + _s[1], s[2] + _s[2]};
    point_xyz_t t3D = {t[0], t[1], t[2]};

    double distance_st = distance( s3D, t3D );
    if ( distance_st > rmdl.get_sensor_range() - EPS )
        return false ;

    return is_visible(s3D, t3D);
}

bool visibility_map::is_antenna_visible( const point_xy_t& a, const point_xy_t& t) const {
    point_xyzt_t _a = rmdl.get_antenna_pose() ; // relative sensor position

    point_xyz_t s3D = {a[0] + _a[0], a[1] + _a[1], _a[2]};
    point_xyz_t t3D = {t[0], t[1], 0};

    double distance_st = distance( s3D, t3D );
    if ( distance_st > rmdl.get_antenna_range() - EPS )
        return false ;

    return is_visible(s3D, t3D);
}

bool visibility_map::is_antenna_visible( const point_xyz_t& a, const point_xyz_t& t) const {
    point_xyzt_t _a = rmdl.get_antenna_pose() ; // relative sensor position

    point_xyz_t s3D = {a[0] + _a[0], a[1] + _a[1], a[2] + _a[2]};
    point_xyz_t t3D = {t[0], t[1], t[2]};

    double distance_st = distance( s3D, t3D );
    if ( distance_st > rmdl.get_antenna_range() - EPS )
        return false ;

    return is_visible(s3D, t3D);
}

/* computing function */
bool visibility_map::is_visible( const point_xyz_t& s3d, const point_xyz_t& t3d) const {//{{{
    // gdalwrap::raster aka. vector<float>
    const auto& heightmap = get_heightmap();
    const auto& npointsmap = get_npointsmap();

    point_xy_t s = {s3d[0], s3d[1]};
    point_xy_t t = {t3d[0], t3d[1]};

    /* Check trivial cases : "s is next to t" or "t out of range" */
    double distance_st = distance( s, t );
    // TODO check if this threshold is valid (robot.radius)
    if ( distance_st < rmdl.get_radius()  + EPS )
        return true ;

    /* Check if both s and t are known (we need zmax !)
     * else we cannot say if they are visible or not,
     * and assume there is no visibility link by default */
    if ( npointsmap[ index(s) ] < 1 - EPS
    ||   npointsmap[ index(t) ] < 1 - EPS)
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
    zs = s3d[2] + heightmap[ index(s) ] ;   // height of the sensor
    zt = t3d[2] + heightmap[ index(t) ] ;           // height of the target
    d0 = distance_st ;

    a = (zs - zt) / d0 ; // d > 0
    // b = 1 and c = -zs

    // test the condition for each point of the line
    for ( auto& p: line) {
        // if p is unknown (never observed), then assume we can see though it.
        if ( npointsmap[ index(p) ] < 1 - EPS )
            continue;
        d = distance( s, p ) ;
        z = heightmap[ index(p) ] ;
        if ( a*d + z - zs > 0 + EPS )
            return false ;
    }

    return true ;
}//}}}


} // namespace gladys
