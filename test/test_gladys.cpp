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

    BOOST_CHECK_EQUAL( gdal_from_file.get_x(),          gdal_to_file.get_x() );
    BOOST_CHECK_EQUAL( gdal_from_file.get_y(),          gdal_to_file.get_y() );
    BOOST_CHECK_EQUAL( gdal_from_file.get_scale_x(),    gdal_to_file.get_scale_x() );
    BOOST_CHECK_EQUAL( gdal_from_file.get_scale_y(),    gdal_to_file.get_scale_y() );
    BOOST_CHECK_EQUAL( gdal_from_file.get_utm_pose_x(), gdal_to_file.get_utm_pose_x() );
    BOOST_CHECK_EQUAL( gdal_from_file.get_utm_pose_y(), gdal_to_file.get_utm_pose_y() );
    BOOST_CHECK_EQUAL( gdal_from_file.bands.size(),     gdal_to_file.bands.size() );
    BOOST_TEST_MESSAGE( "GDAL OK" );
}

BOOST_AUTO_TEST_CASE( test_write_graphviz )
{
    nav_graph obj;
    obj.get_vertex(1,2);
    obj.get_vertex(2,1);
    obj.get_vertex(3,2);
    std::ostringstream oss_graphviz;
    obj.write_graphviz(oss_graphviz);
    BOOST_TEST_MESSAGE( "oss_graphviz.size() = " << oss_graphviz.str().size() );
    BOOST_CHECK_EQUAL( oss_graphviz.str().size() , 102 );
}

BOOST_AUTO_TEST_CASE( test_raster_to_graph )
{
    std::string region_path = "/tmp/test_gladys_raster_to_graph.tif";
    std::string weight_path = "/tmp/test_gladys_raster_to_graph_nav.tif";
    std::string robotm_path = "/tmp/robot.json";
    std::ofstream of(robotm_path);
    of<<"{\"robot\":{\"radius\":1.0,\"velocity\":1.0}}";
    of.close();
    gdal tmp;
    tmp.set_size(5, 4, 4);
    tmp.save(region_path);
    nav_graph ng(region_path, robotm_path);
    std::ostringstream oss_graphviz;
    ng.write_graphviz(oss_graphviz);
    BOOST_TEST_MESSAGE( "oss_graphviz.size() = " << oss_graphviz.str().size() );
    BOOST_CHECK_EQUAL( oss_graphviz.str().size() , 2066 );
    ng.save(weight_path);

    point_xy_t p1 = {1, 1};
    point_xy_t p2 = {2, 2};
    path_t path = ng.astar_search(p1, p2);
    BOOST_TEST_MESSAGE( "test:" + to_string(path) );

    gladys obj(region_path, robotm_path);
}


BOOST_AUTO_TEST_SUITE_END();
