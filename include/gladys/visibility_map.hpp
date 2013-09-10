/*
 * visibility_map.hpp
 *
 * Graph Library for Autonomous and Dynamic Systems
 *
 * author:  Pierrick Koch <pierrick.koch@laas.fr>
 * created: 2013-09-10
 * license: BSD
 */
#ifndef VISIBILITY_MAP_HPP
#define VISIBILITY_MAP_HPP

#include <string>

#include "gladys/gdal.hpp"
#include "gladys/robot_model.hpp"

namespace gladys {

/*
 * from digital terrain map (in multi-layers GeoTiff)
 */
class visibility_map {
    gdal dtm; // probalistic models (multi-layers GeoTiff)
    gdal::raster visibility;
    robot_model rmdl;
public:
    /** load region and robot model
     *
     * @param f_region path to a region.tif file
     * (multi-layers terrains classification probabilities, float32)
     *
     * @param f_robot_model TODO path to a robot model
     * to generate the visibility map (at least its size)
     *
     */
    void load(const std::string& f_dtm, const std::string& f_robot_model) {
        dtm.load(f_dtm);
        rmdl.load(f_robot_model);
        visibility = dtm.get_band("Z_MEAN");
        _load();
    }
    void _load();

    const gdal::raster& get_visibility() const {
        return visibility;
    }

    const gdal& get_dtm() const {
        return dtm;
    }

    size_t get_width() const {
        return dtm.get_width();
    }

    size_t get_height() const {
        return dtm.get_height();
    }

    double get_scale_x() const {
        return dtm.get_scale_x();
    }

    double get_scale_y() const {
        return dtm.get_scale_y();
    }

    double get_utm_pose_x() const {
        return dtm.get_utm_pose_x();
    }

    double get_utm_pose_y() const {
        return dtm.get_utm_pose_y();
    }

    void save(const std::string& filepath) {
        dtm.save(filepath);
    }
};

} // namespace gladys

#endif // VISIBILITY_MAP_HPP

