/*
 * frontier_exploration.cpp
 *
 * Graph Library for Autonomous and Dynamic Systems
 * This class aims at providing a list of frontiers based on the cuurent
 * knowledge of the environment to perform exploration.

 * author:  Cyril Robin <cyril.robin@laas.fr>
 * created: 2013-09-06
 * license: BSD
 */

#include <ostream>      // output stream
#include <stdexcept>    // exceptions
#include <deque>
#include <vector>
#include <cmath>        // for round()

#include "gladys/frontier_exploration.hpp"

#ifndef HEIGHT_CONNEXITY
#define HEIGHT_CONNEXITY
#endif

#ifndef EPS
#define EPS 0.05
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace gladys {

/* frontier_detector class
 ****************************************************************************/

    /* computing functions */
    void frontier_detector::compute_frontiers_WFD(const point_xy_t &_seed) {
        // Get the map
        const weight_map& map = ng.get_map();

        // Get size of the map
        size_t width, height;
        width   = map.get_width();
        height  = map.get_height();
        double x_min, x_max, y_min, y_max;
        x_min = std::max( map.get_utm_pose_x(), x0_area );
        y_min = std::max( map.get_utm_pose_y(), y0_area );
        x_max = std::min( map.get_utm_pose_x() + width, x0_area + height_max ) ;
        y_max = std::min( map.get_utm_pose_y() + height, y0_area + width_max ) ;

        std::cerr   << "[Frontier] (x_min, y_min, x_max, y_max) = ( "
                    << x_min << ", " << y_min << ", "
                    << x_max << ", " << y_max << ") "
                    << std::endl;

        // Get the raster band
        const gdalwrap::raster& data = map.get_weight_band() ;

        // Deal with rounded value (ease the checks for position)
        point_xy_t seed {std::round(_seed[0]), std::round(_seed[1]) };

        std::cerr   << "[Frontier] seed is ("<<seed[0] <<","<<seed[1] 
                    << ") ; index_utm is " << map.index_utm( seed )
                    << " in map (" << width << "," << height << ")." 
                    << std::endl;

        // Check conditions on seed :
        // - within the known area and not an obstacle
        std::cerr   << "[Frontier] data has size : " 
                    << data.size() << std::endl;
        std::cerr   << "[Frontier] data[index_utm(seed)] = " 
                    << data[ map.index_utm( seed ) ] << std::endl;
        if ( data[ map.index_utm( seed ) ] < 0
        ||   map.is_obstacle(data[ map.index_utm( seed ) ]) ){
            throw std::runtime_error("[Frontier] The seed is unknown or \ 
                obstacle : unable to compute frontiers  \
                (and yes, it's a feature! XD )") ;
        }

        /*  compute frontiers with the WFD algorithm
         *
         * Use the description given by :
         * "Robot Exploration with Fast Frontier Detection : Theory and
         * Experiments", M. Keidar afd G. A. Kaminka (AAMAS 2012)
         *
         */
        // Requested containers
        // queues
        std::deque< point_xy_t > mQueue ; // std::queue are restricted std::deque
        std::deque< point_xy_t > fQueue ; // std::queue are restricted std::deque

        // markers lists
        std::vector< bool > mapOpenList       (height*width, false) ;
        std::vector< bool > mapCloseList      (height*width, false) ;
        std::vector< bool > frontierOpenList  (height*width, false) ;
        std::vector< bool > frontierCloseList (height*width, false) ;

        // points
        point_xy_t p,q ;

        //init
        frontiers.empty();    // clear the previous frontiers
        mQueue.push_back( seed );
        mapOpenList[ map.index_utm( seed )] = true ;

        // Main while over queued map points
        while ( !mQueue.empty() ) {

            p = mQueue.front();
            mQueue.pop_front();

            // if p has already been visited, then continue
            if ( mapCloseList[ map.index_utm( p )] ) {
                continue ;
            }

            // else, if p is a frontier point,
            // compute the whole related frontier
            if ( is_frontier( p, x_min, x_max, y_min, y_max, data, map ) ) {

                fQueue.clear();
                // create a new frontier
                frontiers.push_back( points_t() );

                fQueue.push_back( p );
                frontierOpenList[ map.index_utm( p )] = true ;

                // while over potential frontier points
                while ( !fQueue.empty() ) {

                    q = fQueue.front();
                    fQueue.pop_front();

                    // if q has already been visited, then continue
                    if  ( mapCloseList[ map.index_utm( q ) ]
                    || frontierCloseList[ map.index_utm( q ) ]) {
                        continue;
                    }

                    // if p is a frontier point,
                    // deal with it and its neighbours
                    if ( is_frontier( q, x_min, x_max, y_min, y_max, data, map) ) {
                        frontiers.back().push_back( q );
                        //for all neighbours of q
                        for ( auto i : find_neighbours( q, x_min, x_max, y_min, y_max) ) {
                            // if NOT marked yet
                            if  ( !( mapCloseList[ map.index_utm( i ) ]
                            || frontierCloseList[ map.index_utm( i ) ]
                            || frontierOpenList[ map.index_utm( i ) ])) {
                            // then proceed
                                fQueue.push_back( i );
                                frontierOpenList[ map.index_utm( i )] = true ;
                            }
                        }
                    }
                    // mark q
                    frontierCloseList[ map.index_utm( q )] = true ;
                }

                // Note : no need to save the new frontier explicitly

                // mark all points of the new frontier in the closed list
                for ( auto i : frontiers.back() ) {
                    mapCloseList[ map.index_utm( i )] = true ;
                }

            }

            //for all neighbours of p
            for ( auto i : find_neighbours( p, x_min, x_max, y_min, y_max) ) {
                // if NOT marked yet
                if  ( !( mapCloseList[ map.index_utm( i ) ]
                || mapOpenList[ map.index_utm( i ) ])
                // and has at least one neighbour in "Open Space", ie a
                // neighbour in the known area ard which is not an obstacle.
                && ( ! (data[ map.index_utm( i )] < 0                     // unknown
                    || map.is_obstacle(data[ map.index_utm( i )]) ))) {   // obstacle
                    // then proceed
                    mQueue.push_back( i );
                    mapOpenList[ map.index_utm( i )] = true ;
                }
            }

            //mark p
            mapCloseList[ map.index_utm( p )] = true ;
        }
    }

    bool frontier_detector::is_frontier(  const point_xy_t &p,
            //size_t height, size_t width,
            double x_min, double x_max, double y_min, double y_max,
            const gdalwrap::raster& data, const weight_map& map ) 
    {

        // A point is a frontier iff it is in the open space 
        // (i.e. it is know and is not an obstacle )
        if  ( data[ map.index_utm( p ) ] < 0                // unknown
        ||   map.is_obstacle( data[ map.index_utm( p )] ))  //obstacle
            return false ;
        // and at least one of is neighbour is unknown.
        for ( auto i : find_neighbours( p, x_min, x_max, y_min, y_max) )
            if ( data[ map.index_utm( i ) ] < 0 )           // unknown
                return true ;

        return false;
    }

    points_t frontier_detector::find_neighbours( const point_xy_t &p, 
            double x_min, double x_max, double y_min, double y_max )
    {
        points_t neighbours ;

            /* Orientation :
             *
             *  0 → width
             *  ↓
             *  height
             *
             *  NW  N   NE          (-1,-1)     ( 0,-1)     (+1,-1)
             *  W   p   E           (-1, 0)     ( 0, 0)     (+1, 0)
             *  SW  S   SE          (-1,+1)     ( 0,+1)     (+1,+1)
             *
             */

            // North
            if ( p[1] > y_min )
                neighbours.push_back( point_xy_t { p[0]  , p[1]-1 } );
            // South
            if ( p[1] < y_max-1 )
                neighbours.push_back( point_xy_t { p[0]  , p[1]+1 } );
            // East
            if ( p[0] < x_max-1 )
                neighbours.push_back( point_xy_t { p[0]+1, p[1]   } );
            // West
            if ( p[0] > x_min )
                neighbours.push_back( point_xy_t { p[0]-1, p[1]   } );
        #ifdef HEIGHT_CONNEXITY
            // North-East
            if ( p[0] < x_max-1 &&  p[1] > y_min )
                neighbours.push_back( point_xy_t { p[0]+1, p[1]-1 } );
            // Nopth-West
            if ( p[0] > x_min   &&  p[1] > y_min )
                neighbours.push_back( point_xy_t { p[0]-1, p[1]-1 } );
            // South-West
            if ( p[0] > x_min   &&  p[1] < y_max-1 )
                neighbours.push_back( point_xy_t { p[0]-1, p[1]+1 } );
            // South-East
            if ( p[0] < x_max-1 &&  p[1] < y_max-1 )
                neighbours.push_back( point_xy_t { p[0]+1, p[1]+1 } );
        #endif

        return neighbours ;
    }

    void frontier_detector::compute_frontiers(const points_t &r_pos, double yaw,
            size_t max_nf, double min_size, double min_dist, double max_dist,
            algo_t algo )
    {

        assert( r_pos.size() > 0 );


        // try running the algo
        std::cerr   << "[Frontier] Running frontier detection..." << std::endl ;
        switch(algo) {
            case WFD : // Wavefront Frontier Detection
                compute_frontiers_WFD( r_pos[0]) ;
                break;
            case FFD : // Fast Frontier Detection
                throw  std::runtime_error("Fast Frontier Detection is not imlemented yet");
                break;
            default : // Unknown  algorithm
                throw  std::runtime_error("Unknown algorithm for frontier detection");
                break;
        }

        //TODO check if there are any frontier (return something if not)

        // filter frontiers (only keep "promising ones")
        std::cerr   << "[Frontier] Filtering frontiers (among #" 
                    << frontiers.size() << " frontiers)." << std::endl ;
        filter_frontiers(r_pos, max_nf, min_size, min_dist, max_dist ) ;

        //TODO check if there are any frontier left (return something if not)
        
        // compute the frontiers attributes
        std::cerr   << "[Frontier] Computing frontiers attributes..." << std::endl ;
        compute_attributes( r_pos, yaw, min_dist, max_dist );

        std::cerr   << "[Frontier] Done." << std::endl ;

        //TODO return success
        
    }

    void frontier_detector::filter_frontiers(   const points_t& r_pos,
            size_t max_nf, double min_size, double min_dist, double max_dist ) 
    {

        /* init */
        std::vector< points_t > ff ;  // the filtered frontiers fist

        // Quikly compute some "cheap" attributes
        // and filter over them
        for ( unsigned int i = 0 ; i < frontiers.size() ; i++ ) {
            if (frontiers[i].size() < min_size)
                continue; //too small

            // check if a lookout respects distance criterias
            for (auto& pt : frontiers[i] ) {
                double d = distance( r_pos[0], pt) ;
                if ( d > min_dist - EPS && d < max_dist + EPS) {
                    ff.push_back( frontiers[i] ) ;
                    break;
                }
            }
        }

        // filtering over the size if too much frontiers left !
        if (ff.size() > max_nf ) {
            std::sort( ff.begin(), ff.end() ); // asending order using the size
            std::copy( ff.rbegin(), ff.rbegin() + max_nf, frontiers.begin() ) ;
            frontiers.resize( max_nf );
        }
        else 
            frontiers = ff ;

    }

    void frontier_detector::compute_attributes( const points_t &r_pos, 
            double yaw, double min_dist, double max_dist ) 
    {
        /* init */
        size_t total_fPoints = 0 ;
        attributes.resize( frontiers.size() ) ;

        std::cerr   << "[Frontier] Computing attributes for #" << frontiers.size() << " frontiers." << std::endl ;

        /* loop over the frontiers list */
        for ( unsigned int i = 0 ; i < frontiers.size() ; i++ ) {
            //std::cerr   << "[Frontier #"<<i<<"] Computing attributes (size is "<<frontiers[i].size() <<")." << std::endl ;
            attributes[i].ID = i ;
            attributes[i].size = frontiers[i].size() ;
            total_fPoints += frontiers[i].size() ;

            std::cerr   << "[Frontier #"<<i<<"] size is " << attributes[i].size << std::endl ;
            // we arbitrary took the medium point as the lookout
            // TODO Find a better lookout
            // Choose the lookout verifying the distance criterias
            // and minimising the yaw changes
            // Note that there is at least one because of the filter !
            yaw = fmod(yaw, (2*M_PI)); //modulo
            if ( yaw >   M_PI ) yaw -= 2*M_PI; // consider yaw in ]-Pi,Pi]
            if ( yaw <= -M_PI ) yaw += 2*M_PI; // consider yaw in ]-Pi,Pi]
            double min_diff_yaw = 7.0 ;
            for (auto& pt : frontiers[i] ) {
                double d = distance( r_pos[0], pt) ;
                double curr_yaw = yaw_angle_y_inv( r_pos[0], pt) ; // pixel reference uses an inverted y-axis (y -> -y)
                double curr_diff_yaw = yaw - curr_yaw ;
                if ( curr_diff_yaw >   M_PI ) curr_diff_yaw -= 2*M_PI; // consider yaw in ]-Pi,Pi]
                if ( curr_diff_yaw <= -M_PI ) curr_diff_yaw += 2*M_PI; // consider yaw in ]-Pi,Pi]
                curr_diff_yaw = fabs( curr_diff_yaw );
                if ( d > min_dist && d < max_dist 
                &&   curr_diff_yaw < min_diff_yaw ) {
                    min_diff_yaw = curr_diff_yaw ;
                    attributes[i].lookout = pt ;
                    attributes[i].distance  = d ;
                    //break;
                }
            }
            attributes[i].yaw_diff = min_diff_yaw ;
            std::cerr   << "[Frontier #"<<i<<"][lookout] final min_diff_yaw is = " << attributes[i].yaw_diff << std::endl ;
            std::cerr   << "[Frontier #"<<i<<"] lookout is ("<<attributes[i].lookout[0] << "," <<attributes[i].lookout[1] <<")" << std::endl ;

            points_t s, g ;
            path_cost_util_t p ;
            s.push_back( r_pos[0] );
            g.push_back( attributes[i].lookout );
            std::cerr   << "[Frontier #"<<i<<"] Computing path and costs." << std::endl ;
            p = ng.astar_search( s, g) ;
            attributes[i].cost = p.cost ;
            attributes[i].path = p.path ;
            attributes[i].path.push_back(attributes[i].lookout );
            attributes[i].proximity = 0 ; // TODO use utility ! (only one call to A*)
            std::cerr   << "[Frontier #"<<i<<"] A-star 0 = ok." << std::endl ;
            /* the proximity is increased by one for every other robot closer
             * than the focused one */
            for ( unsigned int r = 1 ; r < r_pos.size() ; r++ ) {
                points_t s2 {r_pos[r]} ;
                std::cerr   << "[Frontier #"<<i<<"] A-star " << r << std::endl ;
                double d = ng.astar_search( s2, g).cost ;
                //if ( attributes[i].cost > distance( r_pos[0], r ))
                if ( attributes[i].cost > d )
                    attributes[i].proximity++ ;
            }
        }

        for ( auto& a : attributes )
            a.ratio = (double) a.size / (double) total_fPoints ;
    }

} // namespace gladys
