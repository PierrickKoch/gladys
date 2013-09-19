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
#include <limits> // for numeric_limits::infinity

//#include <boost/graph/adjacency_list.hpp>

#include "gladys/frontier_exploration.hpp"

#ifndef HEIGHT_CONNEXITY
#define HEIGHT_CONNEXITY
#endif

namespace gladys {

/*{{{ frontier_detector class
 ****************************************************************************/

    /* constructors */
    frontier_detector::frontier_detector( const std::string& f_region, const std::string& f_robot_model ) : map( imap ){//{{{
       imap.load( f_region, f_robot_model ) ;
    }//}}}

    frontier_detector::frontier_detector(const weight_map& _map) : map(_map) {}

    /* computing functions */
    void frontier_detector::compute_frontiers_WFD(const point_xy_t &seed) {//{{{
        // Get size of the map
        size_t width, height;
        width   = map.get_width();
        height  = map.get_height();

        // Get the raster band
        const gdal::raster& data = map.get_weight_band() ;

        // check conditions on seed :
        // - inside the map
        assert( seed[0] > 0 && seed[0] < (width-1) && seed[1] > 0 && seed[1] < (height-1) );
        // - within the known area and not an obstacle
        assert( data[ map.idx( seed ) ]  > 0 && data[ map.idx( seed ) ] != std::numeric_limits<float>::infinity() );

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
        mapOpenList[ map.idx( seed )] = true ;

        int c1 = 0;
        // Main while over queued map points
        while ( !mQueue.empty() ) {

            p = mQueue.front();
            mQueue.pop_front();

            // if p has already been visited, then continue
            if ( mapCloseList[ map.idx( p )] ) {
                continue ;
            }

            // else, if p is a frontier point,
            // compute the whole related frontier
            if ( is_frontier( p, height, width, data ) ) {
                int c2 = 0 ;

                fQueue.clear();
                // create a new frontier
                frontiers.push_back( points_t() );

                fQueue.push_back( p );
                frontierOpenList[ map.idx( p )] = true ;

                // while over potential frontier points
                while ( !fQueue.empty() ) {

                    q = fQueue.front();
                    fQueue.pop_front();

                    // if q has already been visited, then continue
                    if  ( mapCloseList[ map.idx( q ) ]
                    || frontierCloseList[ map.idx( q ) ]) {
                        continue;
                    }

                    // if p is a frontier point,
                    // deal with it and its neighbours
                    if ( is_frontier( q, height, width, data ) ) {
                        frontiers.back().push_back( q );
                        //for all neighbours of q
                        for ( auto i : find_neighbours( q, height, width) ) {
                            // if NOT marked yet
                            if  ( !( mapCloseList[ map.idx( i ) ]
                            || frontierCloseList[ map.idx( i ) ]
                            || frontierOpenList[ map.idx( i ) ])) {
                            // then proceed
                                fQueue.push_back( i );
                                frontierOpenList[ map.idx( i )] = true ;
                            }
                        }
                    }
                    // mark q
                    frontierCloseList[ map.idx( q )] = true ;
                }

                // Note : no need to save the new frontier explicitly

                // mark all points of the new frontier in the closed list
                for ( auto i : frontiers.back() ) {
                    mapCloseList[ map.idx( i )] = true ;
                }

            }

            //for all neighbours of p
            for ( auto i : find_neighbours( p, height, width) ) {
                // if NOT marked yet
                if  ( !( mapCloseList[ map.idx( i ) ]
                || mapOpenList[ map.idx( i ) ])
                // and has at least one neighbour in "Open Space", ie a
                // neighbour in the known area ard which is not an obstacle.
                && ( ! (data[ map.idx( i )]  < 0                // unknown
                    || data[ map.idx( i )] == std::numeric_limits<float>::infinity() ))) {   // obstacle
                    // then proceed
                    mQueue.push_back( i );
                    mapOpenList[ map.idx( i )] = true ;
                }
            }

            //mark p
            mapCloseList[ map.idx( p )] = true ;
        }
        //}}}
    }//}}}

    bool frontier_detector::is_frontier(  const point_xy_t &p,     //{{{
                                    size_t height, size_t width,
                                    const gdal::raster& data ) {

        // NB: Remember that index = x + y * width
        
        // A point is a frontier iff it is in the open space 
        // (i.e. it is know and is not an obstacle )
        if ( data[ map.idx( p )]  < 0               // unknown
        ||   data[ map.idx( p )] == std::numeric_limits<float>::infinity() )     // obstacle
            return false ;
        // and at least one of is neighbour is unknown.
        for ( auto i : find_neighbours( p, height, width) )
            if ( data[ map.idx( i ) ] < 0 )          // unknown
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

        //std::cout << "...... #" << neighbours.size() << " neighbours for ("  << p[0] << " , " << p[1] << " ) " << std::endl ;

        return neighbours ;
    }//}}}

    void frontier_detector::compute_frontiers(const point_xy_t &seed, algo_t algo ){//{{{
        // try running the algo
        switch(algo) {
            case WFD : // Wavefront Frontier Detection
                compute_frontiers_WFD( seed ) ;
                break;
            case FFD : // Fast Frontier Detection
                throw  std::runtime_error("Fast Frontier Detection is not imlemented yet");
                break;
            default : // Unknown  algorithm
                throw  std::runtime_error("Unknown algorithm for frontier detection");
                break;
        }

        // compute the frontiers attributes
        compute_attributes( seed );

        // sort the frontiers attributes by the size criteria
        std::sort( attributes.begin(), attributes.end(), std::greater<f_attributes>() ); // descending order

    }//}}}

    void frontier_detector::compute_attributes( const point_xy_t &seed ) {//{{{
        /* init */
        size_t total_fPoints = 0 ;
        attributes.resize( frontiers.size() ) ;

        /* loop over the frontiers list */
        for ( unsigned int i = 0 ; i < frontiers.size() ; i++ ) {
            attributes[i].ID = i ;
            attributes[i].size = frontiers[i].size() ;
            total_fPoints += frontiers[i].size() ;
        }

        for ( auto& a : attributes )
            a.ratio = (double) a.size / (double) total_fPoints ;
    }//}}}

    //void frontier_detector::save_frontiers( const std::string& filepath ) {
    //}

//}}}

} // namespace gladys
