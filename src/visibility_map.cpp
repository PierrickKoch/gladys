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

namespace gladys {

void visibility_map::_load() {//{{{
    width  = dtm.get_width();
    height = dtm.get_height();
    // gdal::raster aka. vector<float>
    const auto& heightmap = dtm.get_band("Z_MAX");
    // TODO
}//}}}

/* computing function */
bool visibility_map::is_visible( const point_xy_t& s, const point_xy_t& t) {//{{{

    /* init */
    bool visible = true ;

    point_xyzt_t _s = rmdl.get_sensor_pos() ; // relative sensor position
    point_xy_t ns ;   // sensor position
    ns[0] = s[0] + _s[0] ; // x
    ns[1] = s[1] + _s[1] ; // y

    /* Check trivial case where s = t */
    if ( s == t )
        return visible ;
    // TODO: check if == is well defined

    // From now, dist( ns, t) > 0
    /* Get the projection of the visibility line with Bresenham's line algorithm */
    points_t line = bresenham( s, t) ;

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
    zs = _s[2] + get_visibility()[ idx(s) ] ;   // height of the sensor
    zt = get_visibility()[ idx(t) ] ;           // height of the target
    d0 = distance( s, t ) ;

    a = (zs - zt) / d0 ; // d > 0
    // b = 1 and c = -zs

    // test the condition for each point of the line
    for ( auto& p: line) {
        d = distance( s, p ) ;
        z = get_visibility()[ idx(p) ] ;
        if ( a*d + z - zs > 0 ) {
            visible = false ;
            break;
        }
    }

    return visible ;
}//}}}


} // namespace gladys
