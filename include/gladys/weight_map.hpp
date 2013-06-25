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

#include <array>
#include <vector>
#include <cassert>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>

#include "gladys/gdal.hpp"

namespace gladys {

/*
 * from terrains model (in multi-layers GeoTiff)
 * to single layers weight map (after inflating obstacles by robot size)
 */
class weight_map {
    gdal terrains; // probalistic models (multi-layers GeoTiff)
    gdal map; // weight map (after inflating robot size)
    /* Names of the visual terrain classes */
    enum {NO_3D_CLASS, FLAT, OBSTACLE, ROUGH, SLOPE, N_RASTER};
    enum {W_UNKNOWN=-2, W_OBSTACLE=-1};
public:
    int load(const std::string filepath) {
        terrains.load(filepath);
        assert(terrains.bands.size() == N_RASTER);
        map.copy_meta(terrains, 1);

        raster data( terrains.bands.size() );
        for (size_t pos = 0; pos < terrains.get_x() * terrains.get_y(); pos++) {
            for (size_t band_id = 0; band_id < data.size(); band_id++)
                data[band_id] = terrains.bands[band_id][pos];
            map.bands[0][pos] = compute_weight(data);
        }
        // TODO inflate obstacle by robot size (get robot model)
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

    size_t get_x() {
        return map.get_x();
    }

    size_t get_y() {
        return map.get_y();
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

