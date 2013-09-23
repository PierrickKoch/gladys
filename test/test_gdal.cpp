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
#include <cmath>
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

BOOST_AUTO_TEST_CASE( test_gdal_scale )
{
    size_t size_x  = 320, // width
           size_y  = 240; // height
    double scale_x = 0.4,
           scale_y = 0.6,
           utm_x   = 123.4,
           utm_y   = 456.7;
    std::string path = "/tmp/test_gladys_gdal.tif";
    gdal obj;
    obj.set_size(2, size_x, size_y);
    obj.set_utm(31); // utm zone
    obj.set_transform(utm_x, utm_y, scale_x, scale_y);

    BOOST_CHECK_EQUAL(obj.idx(0, 0), 0);
    BOOST_CHECK_EQUAL(obj.idx(1, 0), std::ceil(1 / scale_x));
    BOOST_CHECK_EQUAL(obj.idx(0, 1), std::ceil(1 / scale_y * size_x));
    BOOST_CHECK_EQUAL(obj.idx(1, 1), std::ceil(1 / scale_x + 1 / scale_y * size_x));
    BOOST_CHECK_EQUAL(obj.idx(5, 8), std::ceil(5 / scale_x + 8 / scale_y * size_x));
}

BOOST_AUTO_TEST_CASE( test_gdal_scale_utm )
{
    size_t size_x  = 320, // width
           size_y  = 240; // height
    double scale_x = 0.4,
           scale_y = 0.6,
           utm_x   = 123.4,
           utm_y   = 456.7;
    std::string path = "/tmp/test_gladys_gdal.tif";
    gdal obj;
    obj.set_size(2, size_x, size_y);
    obj.set_utm(31); // utm zone
    obj.set_transform(utm_x, utm_y, scale_x, scale_y);

    BOOST_CHECK_EQUAL(obj.idx_utm(utm_x,     utm_y),     0);
    BOOST_CHECK_EQUAL(obj.idx_utm(utm_x + 1, utm_y),     std::ceil(1 / scale_x));
    BOOST_CHECK_EQUAL(obj.idx_utm(utm_x,     utm_y + 1), std::ceil(1 / scale_y * size_x));
    BOOST_CHECK_EQUAL(obj.idx_utm(utm_x + 1, utm_y + 1), std::ceil(1 / scale_x + 1 / scale_y * size_x));
    BOOST_CHECK_EQUAL(obj.idx_utm(utm_x + 5, utm_y + 8), std::ceil(5 / scale_x + 8 / scale_y * size_x));
}

// TODO test custom origin {x,y}

BOOST_AUTO_TEST_SUITE_END();
