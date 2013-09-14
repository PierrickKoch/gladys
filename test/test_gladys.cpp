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

#include "gladys/gdal.hpp"
#include "gladys/gladys.hpp"
#include "gladys/nav_graph.hpp"

BOOST_AUTO_TEST_SUITE( gladys )

BOOST_AUTO_TEST_CASE( test_gdal )
{
    std::string path = "/tmp/test_gladys_gdal.tif";
    gdal gdal_to_file;
    gdal_to_file.set_size(5, 320, 240);
    gdal_to_file.set_utm(31);
    gdal_to_file.set_transform(123, 456, 0.4, 0.6);
    gdal_to_file.save(path);
    // load the file we just saved
    gdal gdal_from_file(path);

    BOOST_CHECK_EQUAL( gdal_from_file, gdal_to_file );
    gdal_from_file.bands[0][0] += 2;
    BOOST_CHECK( !(gdal_from_file == gdal_to_file) );
    BOOST_TEST_MESSAGE( "GDAL OK" );
}

BOOST_AUTO_TEST_CASE( test_write_graphviz )
{
    nav_graph obj;
    obj.new_vertex(1,2);
    obj.new_vertex(2,1);
    obj.new_vertex(3,2);
    std::ostringstream oss_graphviz;
    obj.write_graphviz(oss_graphviz);
    BOOST_TEST_MESSAGE( "oss_graphviz.size() = " << oss_graphviz.str().size() );
    BOOST_CHECK_EQUAL( oss_graphviz.str().size() , 69 );
}

BOOST_AUTO_TEST_CASE( test_raster_to_graph )
{
    std::string region_path = "/tmp/test_gladys_raster_to_graph.tif";
    std::string weight_path = "/tmp/test_gladys_raster_to_graph_nav.tif";
    std::string robotm_path = "/tmp/robot.json";
    std::string graphv_path = "/tmp/test_gladys_raster_to_graph_nav.dot";

    // create a robot model (JSON configuration file)
    std::ofstream robot_cfg(robotm_path);
    robot_cfg<<"{\"robot\":{\"eyez\":1.0,\"mass\":1.0,\"radius\":1.0,\"velocity\":1.0}}";
    robot_cfg.close();

    // create a region map (GeoTiff image)
    gdal region;
    region.set_size(weight_map::N_RASTER, 9, 9);
    // add an obstacle at the center of the map
    region.bands[weight_map::FLAT    ].assign(9*9, 1);
    region.bands[weight_map::FLAT    ][4+4*9] = 0.5;
    region.bands[weight_map::OBSTACLE][4+4*9] = 0.5;
    region.save(region_path);

    // create a navigation graph from the map
    nav_graph ng(region_path, robotm_path);
    std::ostringstream oss_graphviz;
    ng.write_graphviz(oss_graphviz);
    ng.write_graphviz(graphv_path); // for debug
    ng.save(weight_path);

    BOOST_TEST_MESSAGE( "oss_graphviz.size() = " << oss_graphviz.str().size() );
    BOOST_CHECK_EQUAL( oss_graphviz.str().size() , 8812 );

    point_xy_t p1 = {1, 1};
    point_xy_t p2 = {9, 9};
    path_t path = ng.astar_search(p1, p2);
    // g++ >= 4.7 or clang >= 3.1 : support initializer_list
    // TODO BOOST_CHECK_EQUAL( path , {{0.5, 1.0}, {1.0, 1.5}, {1.5, 2.0}} );
    BOOST_TEST_MESSAGE( "path: " + to_string(path) );
    BOOST_CHECK_EQUAL( path.size() , 16 );

    robot_cfg.open(robotm_path);
    robot_cfg<<"{\"robot\":{\"eyez\":1.0,\"mass\":1.0,\"radius\":2.0,\"velocity\":1.0}}";
    robot_cfg.close();
    std::string dtm_path = "/tmp/test_gladys_dtm.tif";
    gdal dtm;
    dtm.set_size(2, 9, 9);
    dtm.bands_name = {"Z_MIN", "Z_MAX"};
    dtm.save(dtm_path);
    gladys obj(region_path, dtm_path, robotm_path);
    points_t start = {p1};
    points_t goal  = {p2};
    motion_constraints_t c;
    path_cost_util_t pcu = obj.navigation(start, goal, 0, c);
    BOOST_TEST_MESSAGE( "path: " + to_string(pcu.path) );
    BOOST_CHECK_EQUAL( path.size() , pcu.path.size() );
}


BOOST_AUTO_TEST_SUITE_END();
