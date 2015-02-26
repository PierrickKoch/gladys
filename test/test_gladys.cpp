/*
 * test_gladys.cpp
 *
 * Test the Graph Library for Autonomous and Dynamic Systems
 *
 * author:  Pierrick Koch <pierrick.koch@laas.fr>
 * created: 2013-07-01
 * license: BSD
 */
#define BOOST_TEST_MODULE const_string test
#include <boost/test/included/unit_test.hpp>

#include <string>
#include <sstream>

#include "gdalwrap/gdal.hpp"
#include "gladys/gladys.hpp"
#include "gladys/nav_graph.hpp"

BOOST_AUTO_TEST_SUITE( gladys )

BOOST_AUTO_TEST_CASE( test_raster_to_graph )
{
    std::string region_path = "/tmp/test_gladys_raster_to_graph.tif";
    std::string weight_path = "/tmp/test_gladys_raster_to_graph_nav.tif";
    std::string robotm_path = "/tmp/robot.json";
    std::string graphv_path = "/tmp/test_gladys_raster_to_graph_nav.dot";

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
    BOOST_CHECK_EQUAL( oss_graphviz.str().size() , 8335 );

    point_xy_t p1 = {1, 1};
    point_xy_t p2 = {5, 9};
    path_t path = ng.astar_search(p1, p2);
    // g++ >= 4.7 or clang >= 3.1 : support initializer_list
    BOOST_TEST_MESSAGE( "path: " + to_string(path) );
    BOOST_CHECK_EQUAL( path.size() , 14 );
    point_xy_t check_point = {1.0, 6.5} ;
    BOOST_CHECK_EQUAL( path[7][0], check_point[0] );
    BOOST_CHECK_EQUAL( path[7][1], check_point[1] );

    point_xy_t p3 = {7, 9};
    points_t goals = {p1, p2, p3};
    std::vector<double> costs = ng.single_source_all_costs(p1, goals);
    BOOST_CHECK_EQUAL( costs[0], ng.astar_search(points_t({p1}), points_t({goals[0]})).cost);
    BOOST_CHECK_EQUAL( costs[1], ng.astar_search(points_t({p1}), points_t({goals[1]})).cost);
    BOOST_CHECK_EQUAL( costs[2], ng.astar_search(points_t({p1}), points_t({goals[2]})).cost);

    robot_cfg.open(robotm_path);
    robot_cfg<<"{\"robot\":{\"mass\":1.0,\"radius\":2.0,\"velocity\":1.0}}";
    robot_cfg.close();
    std::string dtm_path = "/tmp/test_gladys_dtm.tif";
    gdalwrap::gdal dtm;
    dtm.set_size(2, 9, 9);
    dtm.names = {"Z_MIN", "Z_MAX"};
    dtm.save(dtm_path);
    gladys obj(region_path, dtm_path, robotm_path);
    points_t start = {p1};
    points_t goal  = {p2};
    path_cost_util_t pcu = obj.navigation(start, goal);
    BOOST_TEST_MESSAGE( "pcu path: " + to_string(pcu.path) );
    BOOST_CHECK_EQUAL( path.size() , pcu.path.size() );
}


BOOST_AUTO_TEST_SUITE_END();
