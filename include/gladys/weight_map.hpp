/*
 * weight_map.hpp
 *
 * Graph Library for Autonomous and Dynamic Systems
 *
 * author:  Pierrick Koch <pierrick.koch@laas.fr>
 * created: 2013-06-12
 * license: BSD
 */
#ifndef WEIGHT_MAP_HPP
#define WEIGHT_MAP_HPP

#include <string>

#include "gladys/gdal.hpp"
#include "gladys/robot_model.hpp"

namespace gladys {

/*
 * from terrains model (in multi-layers GeoTiff)
 * to single layers weight map (after inflating obstacles by robot size)
 */
class weight_map {
    gdal terrains; // probalistic models (multi-layers GeoTiff)
    gdal map; // weight map (after inflating robot size)
    robot_model rmdl;
    enum {W_FLAG_OBSTACLE=-3, W_UNKNOWN=-2, W_OBSTACLE=-1};
public:
    /* Names of the visual terrain classes */
    enum {NO_3D_CLASS, FLAT, OBSTACLE, ROUGH, SLOPE, N_RASTER};

    /** load region and robot model
     *
     * @param f_region path to a region.tif file
     * (multi-layers terrains classification probabilities, float32)
     *
     * @param f_robot_model TODO path to a robot model
     * to generate the weight map (at least its size)
     *
     */
    void load(const std::string& f_region, const std::string& f_robot_model) {
        terrains.load(f_region);
        rmdl.load(f_robot_model);
        _load();
    }
    void _load() {
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
    void flag_as_obstacle(float& weight) {
        if (!is_obstacle(weight))
            weight = W_FLAG_OBSTACLE;
    }
    bool is_flaged_obstacle(float weight) const {
        return weight == W_FLAG_OBSTACLE;
    }
    bool is_obstacle(float weight) const {
        return weight == W_OBSTACLE;
    }

    float compute_weight(const raster& data) {
        if (data[OBSTACLE] > 0.4) // TODO tune this threshold
            return W_OBSTACLE;
        else // compute a mix of ponderated classes TODO
            return (data[FLAT] * 0.1 + data[ROUGH] * 0.3 + data[SLOPE] * 0.6 );
    }

    raster get_map() {
        return map.bands[0];
    }

    size_t get_width() {
        return map.get_width();
    }

    size_t get_height() {
        return map.get_height();
    }

    double get_scale_x() {
        return map.get_scale_x();
    }

    double get_scale_y() {
        return map.get_scale_y();
    }

    double get_utm_pose_x() {
        return map.get_utm_pose_x();
    }

    double get_utm_pose_y() {
        return map.get_utm_pose_y();
    }

    int save(const std::string filepath) {
        return map.save(filepath);
    }
};

} // namespace gladys

#endif // WEIGHT_MAP_HPP

