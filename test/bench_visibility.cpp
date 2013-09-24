/*
 * bench_visibility.cpp
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

#include "gdalwrap/gdal.hpp"
#include "gladys/visibility_map.hpp"

/**
 * time ./test/bench_visibility ~/sandbox/gladys/tmp/caylusRessacLidar-0.5.dtm.tif ~/sandbox/gladys/tmp/robot.json 1000000
 * 1M points take ~7.6 seconds
 */
int main(int argc, char * argv[])
{
    if (argc < 4) {
        std::cerr<<"usage: "<<argv[0]<<" dtm.tif robot.json N_POINTS"<<std::endl;
        return EXIT_FAILURE;
    }

    gladys::point_xy_t p_start, p_end;
    // create a visibility map from the dtm
    gladys::visibility_map vm;
    vm.load(argv[1], argv[2]);
    double coef_x = static_cast<double>( vm.get_width()  ) / RAND_MAX,
           coef_y = static_cast<double>( vm.get_height() ) / RAND_MAX;

    size_t n_points = std::stoul(argv[3]);
    // std::cout<<"bench "<<n_points<<" points, ["<<coef_x<<","<<coef_y<<"]"<<std::endl;
    std::srand(std::time(0)); //use current time as seed for random generator
    for (size_t i = 0; i < n_points; i++) {
        p_start[0] = coef_x * std::rand();
        p_start[1] = coef_y * std::rand();
        p_end[0]   = p_start[0] + 10.0;
        p_end[1]   = p_start[1] + 10.0;
        vm.is_visible( p_start, p_end ) ;
    }

    return EXIT_SUCCESS;
}
