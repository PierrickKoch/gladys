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

#include <sstream>

#include "gdalwrap/gdal.hpp"
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
    robot_cfg<<"{\"robot\":{\"mass\":1.0,\"radius\":1.0,\"velocity\":1.0}}";
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
     */
    gdalwrap::gdal region;
    region.set_size(weight_map::N_RASTER, 9, 9);
    region.bands[weight_map::FLAT    ].assign(9*9, 1);
    // name bands
    region.names[weight_map::FLAT    ] = "FLAT" ;
    region.names[weight_map::OBSTACLE] = "OBSTACLE";
    region.names[weight_map::NO_3D_CLASS] = "UNKNOWN";
    // add frontiers (top and bottom)
    for ( int i=0 ; i < 9 ; i++ ) {
        region.bands[weight_map::FLAT       ][i    ] = 0. ;
        region.bands[weight_map::NO_3D_CLASS][i    ] = 1  ;
        region.bands[weight_map::FLAT       ][i+8*9] = 0. ;
        region.bands[weight_map::NO_3D_CLASS][i+8*9] = 1  ;
    }
    // add ostacle #1 (middle)
    for ( int i=3 ; i < 6 ; i++ ) {
        region.bands[weight_map::FLAT    ][i+5*9] = 0.2 ;
        region.bands[weight_map::OBSTACLE][i+5*9] = 0.8 ;
    }

    // add ostacle #2 (bottom, centered)
    region.bands[weight_map::FLAT    ][4+8*9] = 0.2 ;
    region.bands[weight_map::OBSTACLE][4+8*9] = 0.8 ;

    region.save(region_path);

    // create a frontier exploration module from the map
    // (Create the weight_map, assumed to be good; cf other unit test)
    weight_map wm ( region_path, robotm_path ) ;
    nav_graph ng ( wm ) ;
    frontier_detector fd ( ng ) ;

    // testing frontier detection with defult algorithm
    point_xy_t r1 {4,4};
    point_xy_t r2 {4,2};
    points_t r_pos {r1, r2} ;
    fd.compute_frontiers( r_pos ) ;

    std::vector< points_t > frontiers = fd.get_frontiers() ;

    //Check the number of frontiers
    BOOST_TEST_MESSAGE( "Nbr of frontiers : frontiers.size() = " << frontiers.size() );
    //BOOST_CHECK_EQUAL( frontiers.size() , 2 );

    size_t c = 0 ;
    std::vector< f_attributes > attributes = fd.get_attributes() ;
    for ( auto& f : attributes )
        c += f.size ;
    BOOST_TEST_MESSAGE( "Nbr of frontier points : c = " << c );
    //BOOST_CHECK_EQUAL( c , 18 );

}

BOOST_AUTO_TEST_SUITE_END();

