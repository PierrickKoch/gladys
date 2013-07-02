/*
 * region_weight.cpp
 *
 * Tool for Graph Library for Autonomous and Dynamic Systems
 *
 * author:  Pierrick Koch <pierrick.koch@laas.fr>
 * created: 2013-07-02
 * license: BSD
 */
#include <ostream> // standard C error stream
#include <cstdlib> // exit status

#include "gladys/nav_graph.hpp"

int main(int argc, char * argv[])
{
    if (argc < 3) {
        std::cerr<<"usage: region_weight region.tif weight.tif"<<std::endl;
        return EXIT_FAILURE;
    }
    gladys::nav_graph ng(argv[1]);//, "/tmp/TODO");
    ng.save(argv[2]);
    return EXIT_SUCCESS;
}
