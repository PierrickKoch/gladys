/*
 * test_gdal.cpp
 *
 * Test the Graph Library for Autonomous and Dynamic Systems
 *
 * author:  Pierrick Koch <pierrick.koch@laas.fr>
 * created: 2013-09-23
 * license: BSD
 */
#define BOOST_TEST_MODULE const_string test
#include <boost/test/included/unit_test.hpp>

#include <string>
#include <sstream>

#include "gladys/gdal.hpp"

BOOST_AUTO_TEST_SUITE( gladys )

BOOST_AUTO_TEST_CASE( test_gdal_equality )
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
    BOOST_TEST_MESSAGE( "GDAL '==' OK" );
}

BOOST_AUTO_TEST_SUITE_END();
