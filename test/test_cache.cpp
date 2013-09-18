/*
 * test_cache.cpp
 *
 * Test the Graph Library for Autonomous and Dynamic Systems
 *
 * author:  Arnaud Degroote <arnaud.degroote@laas.fr>
 * created: 2013-08-18
 * license: BSD
 */
#define BOOST_TEST_MODULE const_string test
#include <boost/test/included/unit_test.hpp>

#include <string>

#include "gladys/cache.hpp"

#include <map>
#include <unordered_map>

BOOST_AUTO_TEST_SUITE( gladys_cache )

namespace { static size_t count_evaluations = 0; }

// Dummy function we want to cache 
std::string invert_string(const std::string& s) 
{ 
  ++count_evaluations; 
  std::string r; 
  std::copy(s.rbegin(), s.rend(), std::back_inserter(r)); 
  return r; 
} 

template <typename Cache>
void main_test()
{
    count_evaluations = 0;

    Cache lru(invert_string, 5);

    // Some initial accesses to prime state 
    BOOST_CHECK_EQUAL(lru("first"), "tsrif"); 
    BOOST_CHECK_EQUAL(lru("second"), "dnoces"); 
    BOOST_CHECK_EQUAL(lru("third"), "driht"); 
    BOOST_CHECK_EQUAL(lru("fourth"), "htruof"); 
    BOOST_CHECK_EQUAL(lru("fifth"), "htfif"); 
    BOOST_CHECK_EQUAL(count_evaluations, 5); 
    BOOST_CHECK_EQUAL(lru("sixth"), "htxis"); 
    BOOST_CHECK_EQUAL(count_evaluations, 6); 
 
    /* 
     * At this point, we expect the cache looks like
     *   second, third, fourth, fifth, sixth
     * We can et second from cache
     */
    BOOST_CHECK_EQUAL(lru("second"), "dnoces"); 
    BOOST_CHECK_EQUAL(count_evaluations, 6); 
 
    /* 
     * At this point, we expect the cache looks like
     *   third, fourth, fifth, sixth, second
     * First is not anymore in the cache, hence the evaluation of
     * count_evaluation
     */
    BOOST_CHECK_EQUAL(lru("first"), "tsrif"); 
    BOOST_CHECK_EQUAL(count_evaluations, 7); 

 
    /* 
     * At this point, we expect the cache looks like
     *   fourth, fifth, sixth, second, first
     * Fourth is expected to be still in the cache
     */
    BOOST_CHECK_EQUAL(lru("fourth"), "htruof"); 
    BOOST_CHECK_EQUAL(count_evaluations, 7); 

    /* 
     * At this point, we expect the cache looks like
     *   fifth, sixth, second, first, fourth
     * seventh must be computed, and will evict fifth
     */
    BOOST_CHECK_EQUAL(lru("seventh"), "htneves"); 
    BOOST_CHECK_EQUAL(count_evaluations, 8); 

    /* 
     * At this point, we expect the cache looks like
     *   sixth, second, first, fourth, seventh
     * check that fifth has been properly evicted
     */
    BOOST_CHECK_EQUAL(lru("fifth"), "htfif"); 
    BOOST_CHECK_EQUAL(count_evaluations, 9); 

    lru.invalidate();

    /* At this point, the cache is expected to empty empty
     * Computation of first must lead to a new evaluation
     */
    BOOST_CHECK_EQUAL(lru("first"), "tsrif"); 
    BOOST_CHECK_EQUAL(count_evaluations, 10); 
}
 
BOOST_AUTO_TEST_CASE( test_cache_map )
{
    main_test<gladys::lru_cache<std::string, std::string, std::map> >();
}

BOOST_AUTO_TEST_CASE( test_cache_unordered_map )
{
    main_test<gladys::lru_cache<std::string, std::string, std::unordered_map> >();
}
BOOST_AUTO_TEST_SUITE_END();
