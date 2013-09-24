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

//#include <string>
//#include <fstream>    // file stream
#include <ostream>      // output stream
#include <stdexcept>    // exceptions
#include <deque>
#include <vector>
#include <limits>       // for numeric_limits::infinity
#include <cmath>        // for round()

//#include <boost/graph/adjacency_list.hpp>

#include "gladys/frontier_exploration.hpp"

#ifndef HEIGHT_CONNEXITY
#define HEIGHT_CONNEXITY
#endif

namespace gladys {

/*{{{ frontier_detector class
 ****************************************************************************/

    /* computing functions */
    void frontier_detector::compute_frontiers_WFD(const point_xy_t &_seed) {//{{{
        // Get the map
        const weight_map& map = ng.get_map();

        // Get size of the map
        size_t width, height;
        width   = map.get_width();
        height  = map.get_height();

        // Get the raster band
        const gdalwrap::raster& data = map.get_weight_band() ;

        // Deal with rounded value (ease the checks for position)
        point_xy_t seed {std::round(_seed[0]), std::round(_seed[1]) };
        
        // Check conditions on seed :
        // - inside the map
        assert( seed[0] > 0 && seed[0] < (width-1) && seed[1] > 0 && seed[1] < (height-1) );
        // - within the known area and not an obstacle
        assert( data[ map.index( seed ) ]  > 0 && data[ map.index( seed ) ] != std::numeric_limits<float>::infinity() );

        /* {{{ compute frontiers with the WFD algorithm
         *
         * Use the description given by :
         * "Robot Exploration with Fast Frontier Detection : Theory and
         * Experiments", M. Keidar afd G. A. Kaminka (AAMAS 2012)
         *
         */
        // Requested containers
        // queues
        std::deque< point_xy_t > mQueue ;       // std::queue are restricted std::deque
        std::deque< point_xy_t > fQueue ;       // std::queue are restricted std::deque

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
        mapOpenList[ map.index( seed )] = true ;

        // Main while over queued map points
        while ( !mQueue.empty() ) {

            p = mQueue.front();
            mQueue.pop_front();

            // if p has already been visited, then continue
            if ( mapCloseList[ map.index( p )] ) {
                continue ;
            }


            // else, if p is a frontier point,
            // compute the whole related frontier
            if ( is_frontier( p, height, width, data, map ) ) {

                fQueue.clear();
                // create a new frontier
                frontiers.push_back( points_t() );

                fQueue.push_back( p );
                frontierOpenList[ map.index( p )] = true ;

                // while over potential frontier points
                while ( !fQueue.empty() ) {

                    q = fQueue.front();
                    fQueue.pop_front();

                    // if q has already been visited, then continue
                    if  ( mapCloseList[ map.index( q ) ]
                    || frontierCloseList[ map.index( q ) ]) {
                        continue;
                    }

                    // if p is a frontier point,
                    // deal with it and its neighbours
                    if ( is_frontier( q, height, width, data, map) ) {
                        frontiers.back().push_back( q );
                        //for all neighbours of q
                        for ( auto i : find_neighbours( q, height, width) ) {
                            // if NOT marked yet
                            if  ( !( mapCloseList[ map.index( i ) ]
                            || frontierCloseList[ map.index( i ) ]
                            || frontierOpenList[ map.index( i ) ])) {
                            // then proceed
                                fQueue.push_back( i );
                                frontierOpenList[ map.index( i )] = true ;
                            }
                        }
                    }
                    // mark q
                    frontierCloseList[ map.index( q )] = true ;
                }

                // Note : no need to save the new frontier explicitly

                // mark all points of the new frontier in the closed list
                for ( auto i : frontiers.back() ) {
                    mapCloseList[ map.index( i )] = true ;
                }

            }

            //for all neighbours of p
            for ( auto i : find_neighbours( p, height, width) ) {
                // if NOT marked yet
                if  ( !( mapCloseList[ map.index( i ) ]
                || mapOpenList[ map.index( i ) ])
                // and has at least one neighbour in "Open Space", ie a
                // neighbour in the known area ard which is not an obstacle.
                && ( ! (data[ map.index( i )]  < 0                // unknown
                    || data[ map.index( i )] == std::numeric_limits<float>::infinity() ))) {   // obstacle
                    // then proceed
                    mQueue.push_back( i );
                    mapOpenList[ map.index( i )] = true ;
                }
            }

            //mark p
            mapCloseList[ map.index( p )] = true ;
        }
        //}}}
    }//}}}

    bool frontier_detector::is_frontier(  const point_xy_t &p,     //{{{
                                    size_t height, size_t width,
                                    const gdalwrap::raster& data, const weight_map& map ) {

        // NB: Remember that index = x + y * width
        
        // A point is a frontier iff it is in the open space 
        // (i.e. it is know and is not an obstacle )
        if ( data[ map.index( p )]  < 0               // unknown
        ||   data[ map.index( p )] == std::numeric_limits<float>::infinity() )     // obstacle
            return false ;
        // and at least one of is neighbour is unknown.
        for ( auto i : find_neighbours( p, height, width) )
            if ( data[ map.index( i ) ] < 0 )          // unknown
                return true ;

        return false;
    }//}}}

    points_t frontier_detector::find_neighbours( const point_xy_t &p, size_t height, size_t width) {//{{{
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
            if ( p[1] > 0 )
                neighbours.push_back( point_xy_t { p[0]  , p[1]-1 } );
            // South
            if ( p[1] < height-1 )
                neighbours.push_back( point_xy_t { p[0]  , p[1]+1 } );
            // East
            if ( p[0] < width-1 )
                neighbours.push_back( point_xy_t { p[0]+1, p[1]   } );
            // West
            if ( p[0] > 0 )
                neighbours.push_back( point_xy_t { p[0]-1, p[1]   } );
        #ifdef HEIGHT_CONNEXITY
            // North-East
            if ( p[0] < width-1 &&  p[1] > 0        )
                neighbours.push_back( point_xy_t { p[0]+1, p[1]-1 } );
            // Nopth-West
            if ( p[0] > 0       &&  p[1] > 0        )
                neighbours.push_back( point_xy_t { p[0]-1, p[1]-1 } );
            // South-West
            if ( p[0] > 0       &&  p[1] < height-1 )
                neighbours.push_back( point_xy_t { p[0]-1, p[1]+1 } );
            // South-East
            if ( p[0] < width-1 &&  p[1] < height-1 )
                neighbours.push_back( point_xy_t { p[0]+1, p[1]+1 } );
        #endif

        return neighbours ;
    }//}}}

    void frontier_detector::compute_frontiers(const points_t &r_pos, //{{{
                            size_t max_nf, size_t min_size,
                            algo_t algo ){

        assert( r_pos.size() > 0 );

        // try running the algo
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

        // filter frontiers (only keep "promising ones")
        filter_frontiers( max_nf, min_size ) ;

        // compute the frontiers attributes
        compute_attributes( r_pos );

    }//}}}

    void frontier_detector::filter_frontiers( size_t max_nf, size_t min_size ) {//{{{
        //TODO
        //std::vector< points_t > ff ;  // the filtered frontiers fist

        //for (auto& f: frontiers)
            //if (f.size() >= min_size )
                //ff.push_back(f) ;

        //frontiers.resize( max_nf ) ;
        //if (ff.size() > max_nf ) {
            //std::sort( ff.begin(), ff.end() ); // asending order using the size
            //std::copy( ff.rbegin(), ff.rbegin() + max_nf, frontiers.begin() ) ;
            //frontiers.resize( max_nf );
        //}
        //else 
            //frontiers = ff ;
    }//}}}

    void frontier_detector::compute_attributes( const points_t &r_pos ) {//{{{
        /* init */
        size_t total_fPoints = 0 ;
        attributes.resize( frontiers.size() ) ;

        /* loop over the frontiers list */
        for ( unsigned int i = 0 ; i < frontiers.size() ; i++ ) {
            attributes[i].ID = i ;
            attributes[i].size = frontiers[i].size() ;
            total_fPoints += frontiers[i].size() ;

            // we arbitrary took the medium point as the lookout
            // TODO Find a better lookout
            attributes[i].lookout   = frontiers[i][ attributes[i].size / 2 ] ;

            points_t s, g ;
            path_cost_util_t p ;
            s.push_back( r_pos[0] );
            g.push_back( attributes[i].lookout );
            //attributes[i].distance  = distance( r_pos[0], attributes[i].lookout ) ;
            p = ng.astar_search( s, g) ;
            attributes[i].distance  = p.cost ;
            attributes[i].path = p.path ;
            attributes[i].proximity = 0 ; // TODO use utility ! (only one call to A*)
            /* the proximity is increased by one for every other robot closer
             * than the focused one */
            for ( auto& r : r_pos ) {
                points_t s2 {r} ;
                double d = ng.astar_search( s2, g).cost ;
                //if ( attributes[i].distance > distance( r_pos[0], r ))
                if ( attributes[i].distance > d )
                    attributes[i].proximity++ ;
            }
        }

        for ( auto& a : attributes )
            a.ratio = (double) a.size / (double) total_fPoints ;
    }//}}}

//}}}

} // namespace gladys
