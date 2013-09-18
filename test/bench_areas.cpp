/*
 * bench_areas.cpp
 *
 * Tool for Graph Library for Autonomous and Dynamic Systems
 *
 * author:  Pierrick Koch <pierrick.koch@laas.fr>
 * created: 2013-09-16
 * license: BSD
 */
#include <ostream> // standard C error stream
#include <cstdlib> // exit status, rand
#include <ctime>   // time
#include <string>  // stoul

#include "gladys/nav_graph.hpp"
#include "gladys/point.hpp"

/**
 * time ./test/bench_areas ~/sandbox/gladys/tmp/caylusRessacLidar-0.5.dtm.tif ~/sandbox/gladys/tmp/robot.json 1000000
 */
int main(int argc, char * argv[])
{
    if (argc < 3) {
        std::cerr<<"usage: "<<argv[0]<<" region.tif robot.json"<<std::endl;
        return EXIT_FAILURE;
    }

    // create a navigation graph from the map
    gladys::nav_graph ng(argv[1], argv[2]);
    std::cout<<gladys::to_string(ng.get_areas())<<std::endl;

    return EXIT_SUCCESS;
}
