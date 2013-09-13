/*
 * test_frontier.cpp
 *
 * Test the Graph Library for Autonomous and Dynamic Systems
 *
 * author:  crobin /TODO
 * created: 2013-09-11
 * license: BSD
 */
#define BOOST_TEST_MODULE const_string test
#include <boost/test/included/unit_test.hpp>

#include <string>
#include <sstream>

#include "gladys/gdal.hpp"
#include "gladys/gladys.hpp"
#include "gladys/nav_graph.hpp"
#include "gladys/frontier_exploration.hpp"

using namespace gladys;

BOOST_AUTO_TEST_SUITE( frontier )

BOOST_AUTO_TEST_CASE( test_frontier )
{
    std::string region_path = "/tmp/test_frontier_detection.tif";
    std::string weight_path = "/tmp/test_frontier_detection_weight.tif";
    std::string robotm_path = "/tmp/robot.json";

    // create a robot model (JSON configuration file)
    std::ofstream robot_cfg(robotm_path);
    robot_cfg<<"{\"robot\":{\"radius\":1.0,\"velocity\":1.0}}";
    robot_cfg.close();

    /* create a region map (GeoTiff image)
     *
     *  U = unknown
     *  F = flat
     *  O = obstacle
     *  S = seed (flat ; initial position)
     *
     *       1 2 3 4 5 6 7 8 9
     *
     *  1    U U U U U U U U U
     *  2    F F F F F F F F F
     *  3    F F F F F F F F F
     *  4    F F F F F F F F F
     *  5    F F F F S F F F F
     *  6    F F F O O O F F F
     *  7    F F F F F F F F F
     *  8    F F F F F F F F F
     *  9    U U U U O U U U U
     *
     *
     *
     */
    gdal region;
    region.set_size(weight_map::N_RASTER, 9, 9);
    region.bands[weight_map::FLAT    ].assign(9*9, 1);
    // add frontiers (top and bottom)
    for ( int i=0 ; i < 9 ; i++ ) {
        region.bands[weight_map::FLAT       ][i    ] = 0. ;
        region.bands[weight_map::NO_3D_CLASS][i    ] = 1  ;
        region.bands[weight_map::FLAT       ][i+8*9] = 0. ;
        region.bands[weight_map::NO_3D_CLASS][i+8*9] = 1  ;
    }
    // add ostacle #1 (middle)
    region.bands[weight_map::FLAT    ][3+5*9] = 0.2 ;
    region.bands[weight_map::OBSTACLE][3+5*9] = 0.8 ;
    region.bands[weight_map::FLAT    ][4+5*9] = 0.2 ;
    region.bands[weight_map::OBSTACLE][4+5*9] = 0.8 ;
    region.bands[weight_map::FLAT    ][5+5*9] = 0.2 ;
    region.bands[weight_map::OBSTACLE][5+5*9] = 0.8 ;
    // add ostacle #2 (bottom, centered)
    region.bands[weight_map::FLAT    ][4+8*9] = 0.2 ;
    region.bands[weight_map::OBSTACLE][4+8*9] = 0.8 ;

    region.save(region_path);

    // create a frontier exploration module from the map
    // (Create the weight_map, assumed to be good; cf other unit test)
    fExploration fExpl ( region_path, robotm_path ) ;

    // testing frontier detection with defult algorithm
    point_xy_t seed {4,4};
    fExpl.compute_frontiers( seed ) ;

    frontiers_list_t fL = fExpl.get_frontiers() ;

    // Check the number of frontiers
    BOOST_TEST_MESSAGE( "Nbr of frontiers : fL.size() = " << fL.size() );
    BOOST_CHECK_EQUAL( fL.size() , 2 );

    size_t c = 0 ;
    for ( auto& f : fL )
        c += f.get_attributes().size ;
    BOOST_TEST_MESSAGE( "Nbr of frontier points : c = " << c );
    BOOST_CHECK_EQUAL( c , 18 );

    //TODO Check on seed position (exception handling)
}


BOOST_AUTO_TEST_SUITE_END();

