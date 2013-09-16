/*
 * test_visibility.cpp
 *
 * Test the Graph Library for Autonomous and Dynamic Systems
 *
 * author:  Cyril Robin <cyril.robin@laas.fr>
 * created: 2013-09-15
 * license: BSD
 */
#define BOOST_TEST_MODULE const_string test
#include <boost/test/included/unit_test.hpp>

#include <string>
#include <sstream>

#include "gladys/gdal.hpp"
#include "gladys/visibility_map.hpp"

BOOST_AUTO_TEST_SUITE( visibility )

BOOST_AUTO_TEST_CASE( test_visibility_map )
{
    std::string dtm_path = "/tmp/test_visibility.tif";
    std::string vm_path = "/tmp/test_visibility_map.tif";
    std::string robotm_path = "/tmp/robot.json";

    // create a robot model (JSON configuration file)
    std::ofstream robot_cfg(robotm_path);
    robot_cfg
        << "{"
            << "\"robot\":{\"mass\":1.0,\"radius\":1.0,\"velocity\":1.0},"
            << "\"sensor\":{\"range\":20.0,\"fov\":6.28,"
                <<   "\"pose\":{\"x\":0.1,\"y\":0.2,\"z\":0.7,\"t\":0.0}"
            << "}"
        << "}" ;
    robot_cfg.close();

    // create a dtm map (GeoTiff image)
    gladys::gdal dtm;
    dtm.set_size(1, 9, 9);
    // add a small wall in the middle of the map
    dtm.bands_name[0] = "Z_MAX";
    dtm.get_band("Z_MAX").assign(9*9, 0.5);
    for (int i=0 ; i<9 ; i++ )
        dtm.get_band("Z_MAX")[ 5 + i*9 ] = 1.3;
    // two special points to observe
    dtm.get_band("Z_MAX")[ 8 + 0*9 ] = 1.9;
    dtm.get_band("Z_MAX")[ 8 + 8*9 ] = 1.1;
    dtm.get_band("Z_MAX")[ 0 + 5*9 ] = 0.6;
    dtm.save(dtm_path);

    //// create a visibility map from the dtm
    gladys::visibility_map vm;
    vm.load(dtm_path, robotm_path);
    vm.save(vm_path);

    gladys::point_xy_t s  = {0, 5}; // sensor
    gladys::point_xy_t t1 = {8, 0}; // target 1 -- visible
    gladys::point_xy_t t2 = {8, 8}; // target 2 -- invisible

    bool b ;
    b = vm.is_visible( s, t1) ;
    BOOST_TEST_MESSAGE( "t1 visible from s ? " + b );
    BOOST_CHECK_EQUAL( b, true );

    b = vm.is_visible( s, t2) ;
    BOOST_TEST_MESSAGE( "t2 visible from s ? " + b );
    BOOST_CHECK_EQUAL( b, false );

}

BOOST_AUTO_TEST_SUITE_END();

