/*
 * weight_map.cpp
 *
 * Graph Library for Autonomous and Dynamic Systems
 *
 * author:  Pierrick Koch <pierrick.koch@laas.fr>
 * created: 2013-07-18
 * license: BSD
 */
#include <cmath>
#include <map>

#include "gdalwrap/gdal.hpp"
#include "gladys/weight_map.hpp"

namespace gladys {

void costmap::_load() {
    assert(terrains.bands.size() > 1);
    map.copy_meta(terrains, 1);
    gdalwrap::raster& weights = map.bands[0];
    width = map.get_width();
    map.names[0] = "WEIGHT";
    for (size_t pos = 0; pos < width * map.get_height(); pos++) {
        if (terrains.bands[1][pos] < 100) // confidence bellow 100 -> unknown
            weights[pos] = W_UNKNOWN;
        else if (terrains.bands[0][pos] > 252) // TODO tune this threshold
            weights[pos] = std::numeric_limits<float>::infinity(); // obstacle
        else {
            weights[pos] = terrains.bands[0][pos] / rmdl.get_velocity();
        }
    }
}

/** compute a mix of ponderated classes
 *
 * w/ threshold on obstacle and unknown
 * @returns weight from probabilities ponderated by robot model
 *          (in seconds per meter)
 */
void weight_map::_load() {
    assert(terrains.bands.size() > 1);
    map.copy_meta(terrains, 1);
    gdalwrap::raster& weights = map.bands[0];
    width = map.get_width();
    map.names[0] = "WEIGHT";

    float weight = 1.0;
    std::map<std::string, float> costs = rmdl.get_costs();
    size_t idx_no_3d_class = terrains.get_band_id("NO_3D_CLASS");
    size_t idx_obstacle    = terrains.get_band_id("OBSTACLE");

    for (size_t pos = 0; pos < width * map.get_height(); pos++) {
        if (terrains.bands[idx_no_3d_class][pos] > 0.9)
            weights[pos] = W_UNKNOWN; // UNKNOWN
        else if (terrains.bands[idx_obstacle][pos] > 0.4) // TODO tune this threshold
            weights[pos] = std::numeric_limits<float>::infinity(); // OBSTACLE
        else {
            weight = 1.0;
            // compute a mix of ponderated classes TODO dynamicaly json conf
            for (const auto& kv : costs)
                weight += kv.second * terrains.get_band(kv.first)[pos];

            weights[pos] = weight / rmdl.get_velocity();
        }
    }

#if 0
    // inflate obstacle by robot radius relative to map scale {x,y}

    // averaged up raduis {x,y}
    size_t rx = std::floor( map.get_scale_x() * rmdl.get_radius() );
    size_t ry = std::floor( map.get_scale_y() * rmdl.get_radius() );

    for (size_t px_x = rx; px_x < width  - rx; px_x++)
    for (size_t px_y = ry; px_y < map.get_height() - ry; px_y++) {
        if ( is_obstacle(weights[px_x + px_y * width]) ) {
            for (size_t irx = 1; irx <= rx; irx++)
            for (size_t iry = 1; iry <= ry; iry++) {
                flag_as_obstacle(weights[px_x       + (px_y - iry) * width]);
                flag_as_obstacle(weights[px_x       + (px_y + iry) * width]);
                flag_as_obstacle(weights[px_x - irx + (px_y      ) * width]);
                flag_as_obstacle(weights[px_x - irx + (px_y - iry) * width]);
                flag_as_obstacle(weights[px_x - irx + (px_y + iry) * width]);
                flag_as_obstacle(weights[px_x + irx + (px_y      ) * width]);
                flag_as_obstacle(weights[px_x + irx + (px_y - iry) * width]);
                flag_as_obstacle(weights[px_x + irx + (px_y + iry) * width]);
            }
        }
    }
    // see http://www.ros.org/wiki/costmap_2d#Inflation
    for (auto& weight : weights)
        if (is_flag_obstacle(weight))
            weight = 99;
#endif

}

} // namespace gladys
