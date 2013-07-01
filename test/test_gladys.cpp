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

#include "gladys/gladys.hpp"
#include "gladys/nav_graph.hpp"

BOOST_AUTO_TEST_CASE( test_write_graphviz )
{
    gladys::nav_graph obj;
    obj.get_vertex(1,2);
    obj.get_vertex(2,1);
    obj.get_vertex(3,2);
    obj.write_graphviz();
}

BOOST_AUTO_TEST_CASE( test_raster_to_graph )
{
    std::string path = "/tmp/gladys.tif";
    gladys::gdal tmp;
    tmp.set_size(5, 4, 4);
    tmp.save(path);
    gladys::nav_graph ng(path);
    ng.write_graphviz();
    ng.save("/tmp/gladys_nav_graph.tif");

    gladys::gladys obj(path, "/tmp/TODO");
}

