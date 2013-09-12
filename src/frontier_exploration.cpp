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

//#include <boost/graph/adjacency_list.hpp>

#include "gladys/frontier_exploration.hpp"

#ifndef HEIGHT_CONNEXITY
#define HEIGHT_CONNEXITY
#endif

namespace gladys {

/*{{{ frontier class
 ****************************************************************************/

    /* constructors */
    Frontier::Frontier() {}

    Frontier::Frontier(const points_t &_points) : points(_points) {}

    /* computing functions */
    void Frontier::compute_attributes() {//{{{
        //TODO
        size = points.size();
        distance = 1.0;
    }//}}}

    /* setters */
    void Frontier::add_point(const point_xy_t _point) {//{{{
        points.push_back( _point );
    }//}}}

    /* getters */
    const points_t& Frontier::get_points() {//{{{
        return points ;
    }//}}}

//}}}

/*{{{ fExploration class
 ****************************************************************************/

    /* constructors */
    fExploration::fExploration( const std::string& f_region, const std::string& f_robot_model ) : map( imap ){//{{{
       imap.load( f_region, f_robot_model ) ;
    }//}}}

    fExploration::fExploration(const weight_map& _map) : map(_map) {}

    /* computing functions */
    void fExploration::compute_frontiers_WFD(const point_xy_t seed) {//{{{
        // Get size of the map
        size_t width, height;
        width   = map.get_width();
        height  = map.get_height();

        // Get the raster band
        const gdal::raster& data = map.get_weight_band() ;

        //bool areThereFrontiers = false ;
        //for (auto& p : data )
            //if (p < 0) { // unknown area
                //areThereFrontiers = true ;
                //break;
            //}
        //if ( areThereFrontiers )
            //std::cout << "There are indeed unknown areas" << std::endl ;
        //else {
            //std::cout << "There are NO unknown area" << std::endl ;
            //exit;
        //}

        // check conditions on seed :
        // (NB: seed uses double values)
        // - inside the map
        if ( seed[0] < 0 || seed[0] > (width-1)
        ||   seed[1] < 0 || seed[1] > (height-1) )
            throw  std::runtime_error("[WFD] Seed outside of the area.");

        // - within the known area and not an obstacle
        // (Remember : index = x + y * width)
        if ( data[ seed[0] + seed[1] * width ]  < 0             // unknown
        ||   data[ seed[0] + seed[1] * width ] == HUGE_VALF )   // obstacle
            throw  std::runtime_error("[WFD] Seed in unknown area.");

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
        fList.empty();    // clear the previous frontiers
        mQueue.clear();
        mQueue.push_back( seed );
        mapOpenList.at( seed[0] + seed[1] * width);

        int c1 = 0;
        // Main while over queued map points
        while ( !mQueue.empty() ) {
            //std::cout << "#" << c1++ ;

            p = mQueue.front();
            mQueue.pop_front();

            //std::cout << " (" << p[0] << " , " << p[1] << " ) " ;

            // if p has already been visited, then continue
            if ( mapCloseList.at( p[0] + p[1] * width ) ) {
                //std::cout << "***** continue #1 !" << std::endl ;
                continue ;
            }

            // else, if p is a frontier point,
            // compute the whole related frontier
            if ( isFrontier( p, height, width, data ) ) {
                //std::cout << "is a frontier point…" << std::endl ;
                int c2 = 0 ;

                fQueue.clear();
                // create a new frontier
                fList.push_back( Frontier() );

                fQueue.push_back( p );
                frontierOpenList.at( p[0] + p[1] * width ) = true ;

                // while over potential frontier points
                while ( !fQueue.empty() ) {
                    //std::cout << "*Building frontier : #" << c2++ ;

                    q = fQueue.front();
                    fQueue.pop_front();
                    //std::cout << " (with ( " << q[0] << "," << q[1] << " )" << std::endl ;

                    // if q has already been visited, then continue
                    if  ( mapCloseList.at( q[0] + q[1] * width )
                    || frontierCloseList.at( q[0] + q[1] * width ) ) {
                        //std::cout << "***** continue #2 !" << std::endl ;
                        continue;
                    }

                    // if p is a frontier point,
                    // deal with it and its neighbours
                    if ( isFrontier( q, height, width, data ) ) {
                        //fQueue.push_back( q );
                        fList.back().add_point( q );
                        //std::cout << "* I've added a frontier point…" << std::endl ;
                        //for all neighbours of q
                        for ( auto i : findNeighbours( q, height, width) ) {
                            //std::cout << "*** for #1" << std::endl ;
                            // if NOT marked yet
                            if  ( !( mapCloseList.at( i[0] + i[1] * width )
                            || frontierCloseList.at( i[0] + i[1] * width )
                            || frontierOpenList.at( i[0] + i[1] * width ) )) {
                            // then proceed
                                //std::cout << "*** ( " << i[0] << " , " << i[1] << " ) is a candidate" << std::endl ;
                                fQueue.push_back( i );
                                frontierOpenList.at( i[0] + i[1] * width ) = true ;
                            }
                        }
                    }
                    // mark q
                    frontierCloseList.at( q[0] + q[1] * width ) = true ;
                }

                // Note : no need to save the new frontier explicitly

                //std::cout << "***** plop !" << std::endl ;
                // mark all points of the new frontier in the closed list
                for ( auto i : fList.back().get_points() ) {
                    //std::cout << "*** for #2" << std::endl ;
                    mapCloseList.at( i[0] + i[1] * width ) = true ;
                }

            }
            //else
                //std::cout << "is NOT a frontier point…" << std::endl ;

            //std::cout << "*** Add neighbours" << std::endl ;
            //for all neighbours of p
            for ( auto i : findNeighbours( p, height, width) ) {
                //std::cout << "*** for #3" << std::endl ;

                // if NOT marked yet
                if  ( !( mapCloseList.at( i[0] + i[1] * width )
                || mapOpenList.at( i[0] + i[1] * width ) )
                // and has at least one neighbour in "Open Space", ie a
                // neighbour in the known area ard which is not an obstacle.
                //&& hasOpenSpaceNeighbour( i, height, width, data ) ) {
                && ( ! (data[ i[0] + i[1] * width ]  < 0                // unknown
                    || data[ i[0] + i[1] * width ] == HUGE_VALF ))) {   // obstacle
                // then proceed
                    mQueue.push_back( i );
                    mapOpenList.at( i[0] + i[1] * width ) = true ;
                    //std::cout << "*** add : ("  << i[0] << " , " << i[1] << " ) " << std::endl ;
                }
            }

            //mark p
            mapCloseList.at( p[0] + p[1] * width ) = true ;
        }
        //}}}
    }//}}}

    bool fExploration::isFrontier(  const point_xy_t p,     //{{{
                                    size_t height, size_t width,
                                    const gdal::raster& data ) {

        // NB: Remember that index = x + y * width
        
        // A point is a frontier iff it is in the open space 
        // (i.e. it is know and is not an obstacle )
        if ( data[ p[0] + p[1] * width ]  < 0               // unknown
        ||   data[ p[0] + p[1] * width ] == HUGE_VALF )     // obstacle
            return false ;
        // and at least one of is neighbour is unknown.
        for ( auto i : findNeighbours( p, height, width) )
            if ( data[ i[0] + i[1] * width ] < 0 )          // unknown
                return true ;

        return false;
    }//}}}

    bool fExploration::hasOpenSpaceNeighbour(   const point_xy_t p,  //{{{
                                                size_t height, size_t width,
                                                const gdal::raster& data ) {

        // NB: Remember that index = x + y * width
        
        // A point is in the open space iff it is know and is not an obstacle.
        // Here at least one open space neighbour is required.
        for ( auto i : findNeighbours( p, height, width) )
            if ( ! (data[ i[0] + i[1] * width ]  < 0           // unknown
                || data[ i[0] + i[1] * width ] == HUGE_VALF )) // obstacle
                return true ;

        return false;
    }//}}}

    points_t fExploration::findNeighbours( const point_xy_t p, size_t height, size_t width) {//{{{
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

    void fExploration::compute_frontiers(const point_xy_t seed, const algo_t algo ){//{{{
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
        compute_attributes();

    }//}}}

    void fExploration::compute_attributes() {//{{{
        // loop over the frontiers list
        for (auto f : fList)
            f.compute_attributes();
    }//}}}

    //void fExploration::save_frontiers( const std::string& filepath ) {
    void fExploration::save_frontiers() {
        std::cout << " Oh yeah !!" << std::endl ;
    }

    /* getters */
    const weight_map& fExploration::get_map() const {//{{{
        return map;
    }//}}}
    const frontiers_list_t& fExploration::get_frontiers() const {//{{{
        return fList;
    }//}}}

//}}}

} // namespace gladys
