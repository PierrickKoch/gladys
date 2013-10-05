/*
 * test_dstar.cpp
 *
 * Test the Graph Library for Autonomous and Dynamic Systems
 *
 * author:  Arnaud Degroote <arnaud.degroote@laas.fr>
 * created: 2013-09-10
 * license: BSD
 */
#define BOOST_TEST_MODULE const_string test
#include <boost/test/included/unit_test.hpp>

#include <string>
#include <sstream>

#include "gdalwrap/gdal.hpp"
#include "gladys/gladys.hpp"
#include "gladys/nav_graph.hpp"
#include "gladys/dstar.hpp"

using namespace gladys;

BOOST_AUTO_TEST_SUITE( dstar )

BOOST_AUTO_TEST_CASE( test_dstar )
{
    std::string region_path = "/tmp/test_dstar_raster_to_graph.tif";
    std::string weight_path = "/tmp/test_dstar_raster_to_graph_nav.tif";
    std::string robotm_path = "/tmp/robot.json";
    std::string graphv_path = "/tmp/test_dstar_raster_to_graph_nav.dot";

    // create a robot model (JSON configuration file)
    std::ofstream robot_cfg(robotm_path);
    robot_cfg<<"{\"robot\":{\"mass\":1.0,\"radius\":1.0,\"velocity\":1.0}}";
    robot_cfg.close();

    // create a region map (GeoTiff image)
    gdalwrap::gdal region;
    region.set_size(4, 9, 9);
    // name bands
    region.names = {"NO_3D_CLASS", "FLAT", "OBSTACLE", "ROUGH"};
    // add an obstacle at the center of the map
    region.bands[1].assign(9*9, 1);
    for ( int i=1 ; i < 9 ; i++ ) {
        region.bands[1][i+5*9] = 0.2 ;
        region.bands[2][i+5*9] = 0.8 ;
    }
    region.save(region_path);

    // create a navigation graph from the map
    weight_map wm(region_path, robotm_path);
    nav_graph ng(wm);
    std::ostringstream oss_graphviz;
    ng.write_graphviz(oss_graphviz);
    ng.write_graphviz(graphv_path); // for debug
    ng.save(weight_path);

    BOOST_TEST_MESSAGE( "oss_graphviz.size() = " << oss_graphviz.str().size() );
    BOOST_CHECK_EQUAL( oss_graphviz.str().size() , 8812 );

    point_xy_t p1 = {1, 1};
    point_xy_t p2 = {5, 9};
    dstar_search dstar(ng.get_graph(), ng.get_closest_vertex(p1), ng.get_closest_vertex(p2));
    path_t path = dstar.get_path();
    BOOST_TEST_MESSAGE( "path: " + to_string(path) );
    BOOST_CHECK_EQUAL( path.size() , 14 );
    point_xy_t check_point = {1.0, 6.5} ;
    BOOST_CHECK_EQUAL( path[7][0], check_point[0] );
    BOOST_CHECK_EQUAL( path[7][1], check_point[1] );


    // replan must be instant, and do not change the plan
    dstar.replan(ng.get_closest_vertex(p1));
    path = dstar.get_path();
    BOOST_TEST_MESSAGE( "path: " + to_string(path) );
    BOOST_CHECK_EQUAL( path.size() , 14 );

    dstar.replan(ng.get_closest_vertex(path[2]));
    path = dstar.get_path();
    BOOST_TEST_MESSAGE( "path: " + to_string(path) );
    BOOST_CHECK_EQUAL( path.size() , 12 );

}

BOOST_AUTO_TEST_SUITE_END();
