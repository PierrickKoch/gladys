/*
 * test_bresenham.cpp
 *
 * Test the Graph Library for Autonomous and Dynamic Systems
 *
 * author:  Cyril Robin <cyril.robin@laas.fr>
 * created: 2013-09-13
 * license: BSD
 */
#define BOOST_TEST_MODULE const_string test
#include <boost/test/included/unit_test.hpp>

#include "gladys/bresenham.hpp"

using namespace gladys;

BOOST_AUTO_TEST_SUITE( bresenham )

BOOST_AUTO_TEST_CASE( test_bresenham )
{
    // two arbitrary points
    gladys::point_xy_t s {  1, 1 } ;
    gladys::point_xy_t t { 11, 5 } ;
    
    // ascending line
    gladys::points_t l = gladys::bresenham( s, t );

    BOOST_TEST_MESSAGE( "line 1 size = " << l.size() );
    BOOST_CHECK_EQUAL( l.size() , 11 );

    // descending line
    gladys::points_t m = gladys::bresenham( t, s );
    BOOST_TEST_MESSAGE( "line 2 size = " << m.size() );
    BOOST_CHECK_EQUAL( m.size() , 11 );

    // m is the reverse of l
    bool b = true;
    for ( unsigned int i = 0 ; ( b && ( i < 11 )) ; i++ )
        b = ( l[i][0] == m[10-i][0] && l[i][1] == m[10-i][1] ) ;
    BOOST_TEST_MESSAGE( "Check order " );
    BOOST_CHECK_EQUAL( b , true );

    // Check one specific point
    b ==  ( l[5][0] == 3 && l[5][1] == 5 ) ;
    BOOST_TEST_MESSAGE( "Check the coordinate of some specific point" );
    BOOST_CHECK_EQUAL( b , true );
}

BOOST_AUTO_TEST_SUITE_END();
