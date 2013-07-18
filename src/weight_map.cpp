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

#include "gladys/gdal.hpp"
#include "gladys/weight_map.hpp"

namespace gladys {

void weight_map::_load() {
    assert(terrains.bands.size() == N_RASTER);
    map.copy_meta(terrains, 1);
    raster& weights = map.bands[0];

    raster data( terrains.bands.size() );
    for (size_t pos = 0; pos < map.get_width() * map.get_height(); pos++) {
        for (size_t band_id = 0; band_id < data.size(); band_id++)
            data[band_id] = terrains.bands[band_id][pos];
        weights[pos] = compute_weight(data);
    }

    // inflate obstacle by robot radius relative to map scale {x,y}

    // averaged up raduis {x,y}
    size_t rx = std::floor( map.get_scale_x() * rmdl.get_radius() );
    size_t ry = std::floor( map.get_scale_y() * rmdl.get_radius() );

    for (size_t px_x = 1; px_x < map.get_width() - 1; px_x++)
    for (size_t px_y = 1; px_y < map.get_height() - 1; px_y++) {
        if ( is_obstacle(weights[px_x + px_y * map.get_width()]) ) {
            for (size_t irx = 0; irx <= rx; irx++)
            for (size_t iry = 0; iry <= ry; iry++) {
                flag_as_obstacle(weights[px_x       + (px_y - iry) * map.get_width()]);
                flag_as_obstacle(weights[px_x       + (px_y + iry) * map.get_width()]);
                flag_as_obstacle(weights[px_x - irx + (px_y      ) * map.get_width()]);
                flag_as_obstacle(weights[px_x - irx + (px_y - iry) * map.get_width()]);
                flag_as_obstacle(weights[px_x - irx + (px_y + iry) * map.get_width()]);
                flag_as_obstacle(weights[px_x + irx + (px_y      ) * map.get_width()]);
                flag_as_obstacle(weights[px_x + irx + (px_y - iry) * map.get_width()]);
                flag_as_obstacle(weights[px_x + irx + (px_y + iry) * map.get_width()]);
            }
        }
    }
    for (auto& weight : weights)
        if (is_flaged_obstacle(weight))
            weight = W_OBSTACLE;

    // TODO reduce scale to rmdl.get_radius() boost/gil/extension/numeric/resample.hpp
}

} // namespace gladys
