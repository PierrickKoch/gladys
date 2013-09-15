/*
 * bresenham.cpp
 *
 * Graph Library for Autonomous and Dynamic Systems
 *
 * author:  Cyril Robin <cyril.robin@laas.fr>
 * created: 2013-09-13
 * license: BSD
 */

#include <cmath>

#include "gladys/bresenham.hpp"

namespace gladys {

points_t bresenham( const point_xy_t& s, const point_xy_t& t) {//{{{

/* Bresenham's algorithm implemented here :
 *
 * function line(x0, y0, x1, y1)
 *      boolean steep := abs(y1 - y0) > abs(x1 - x0)
 *      if steep then
 *          swap(x0, y0)
 *          swap(x1, y1)
 *      int deltax := abs(x1 - x0)
 *      int deltay := abs(y1 - y0)
 *      int error := deltax / 2
 *      int ystep
 *      int y := y0
 *
 *      int inc REM added
 *      if x0 < x1 then inc := 1 else inc := -1
 *
 *      if y0 < y1 then ystep := 1 else ystep := -1
 *      for x from x0 to x1 with increment inc
 *          if steep then plot(y,x) else plot(x,y)
 *          increment here a variable to control the progress of the line drawing
 *          error := error - deltay
 *          if error < 0 then
 *              y := y + ystep
 *              error := error + deltax
 *
 */

    /* init */
    points_t line ;
    int x0 = s[0] ;
    int y0 = s[1] ;
    int x1 = t[0] ;
    int y1 = t[1] ;

    bool steep = ( abs(y1 - y0) > abs(x1 - x0) ) ;
    if ( steep ) {
        std::swap(x0, y0) ;
        std::swap(x1, y1) ;
    }

    int deltax = abs(x1 - x0) ;
    int deltay = abs(y1 - y0) ;
    int error = deltax / 2 ;
    int y = y0 ;

    int ystep   = ( y0 < y1 ? 1 : -1 );

    /* line */
    if ( x0 < x1 ) // ascending
        for ( int x = x0 ; x <= x1 ; x++ ) { 
            if ( steep )
                line.push_back( point_xy_t { (double) y, (double) x } ) ;
            else
                line.push_back( point_xy_t { (double) x, (double) y } ) ;
            // progression control
            error -= deltay ;
            if ( error < 0 ) {
                y += ystep ;
                error += deltax ;
            }
        }
    else //descending
        for ( int x = x0 ; x >= x1 ; x-- ) { 
            if ( steep )
                line.push_back( point_xy_t { (double) y, (double) x } ) ;
            else
                line.push_back( point_xy_t { (double) x, (double) y } ) ;
            // progression control
            error -= deltay ;
            if ( error < 0 ) {
                y += ystep ;
                error += deltax ;
            }
        }
    /* result */
    return line ;
}//}}}

} // namespace gladys
