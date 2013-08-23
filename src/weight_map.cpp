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
#include <fstream>
#include <stdexcept>        // for runtime_error

#include "gladys/gdal.hpp"
#include "gladys/weight_map.hpp"

namespace gladys {

void weight_map::_load() {
    assert(terrains.bands.size() == N_RASTER);
    map.copy_meta(terrains, 1);
    gdal::raster& weights = map.bands[0];
    size_t width = map.get_width();

    gdal::raster data( terrains.bands.size() );
    for (size_t pos = 0; pos < width * map.get_height(); pos++) {
        for (size_t band_id = 0; band_id < data.size(); band_id++)
            data[band_id] = terrains.bands[band_id][pos];
        weights[pos] = compute_weight(data);
    }

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
}

void weight_map::save_pgm(const std::string& filepath) {
    const gdal::raster& weights = map.bands[0];
    std::ofstream file(filepath);
    if (!file.is_open())
        throw std::runtime_error("[weight_map::save_pgm] could not open filepath");

    file<<"P2\n"<<map.get_width()<<'\n'<<map.get_height()<<'\n'<<0xff<<'\n';
    for (size_t px_x = 0; px_x < map.get_width(); px_x++) {
        for (size_t px_y = 0; px_y < map.get_height(); px_y++) {
            unsigned char cost = (unsigned char)weights[px_x + px_y * map.get_width()];
            file<<cost<<' ';
        }
        file<<'\n';
    }
    file.close();
}

} // namespace gladys
