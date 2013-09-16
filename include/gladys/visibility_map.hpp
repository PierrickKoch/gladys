/*
 * visibility_map.hpp
 *
 * Graph Library for Autonomous and Dynamic Systems
 *
 * author:  Pierrick Koch <pierrick.koch@laas.fr>
 *          Cyril Robin <cyril.robin@laas.fr>
 * created: 2013-09-10
 * license: BSD
 */
#ifndef VISIBILITY_MAP_HPP
#define VISIBILITY_MAP_HPP

#include <string>

#include "gladys/gdal.hpp"
#include "gladys/robot_model.hpp"
#include "gladys/point.hpp"

namespace gladys {

/*
 * from digital terrain map (in multi-layers GeoTiff)
 */
class visibility_map {
    gdal dtm; // digital terrain map (multi-layers GeoTiff)
    robot_model rmdl;
    size_t width;  // dtm width
    size_t height; // dtm height

    void _load();

public:
    /** load region and robot model
     *
     * @param f_region path to a region.tif file
     * (multi-layers terrains classification probabilities, float32)
     *
     * @param f_robot_model path to a robot model
     * to generate the visibility map (at least its sensor height)
     *
     */
    void load(const std::string& f_dtm, const std::string& f_robot_model) {
        dtm.load(f_dtm);
        rmdl.load(f_robot_model);
        _load();
    }

    /* computing function */

    /** test if point 't' (target) is visible from 's' (sensor)
     *
     * @param s the position of the sensor
     *
     * @param t the position of the target
     *
     * @returns true if visible.
     *
     */
    bool is_visible( const point_xy_t& s, const point_xy_t& t) const ;

    /** Get the index of the point in the raster
     *
     * @param p the point
     *
     * @returns the index of the point
     *
     */
    double idx( const point_xy_t& p ) const {
        return p[0] + p[1] * width;
    }

    /* getters */
    const gdal::raster& get_heightmap() const {
        return dtm.get_band("Z_MAX");
    }

    const gdal& get_dtm() const {
        return dtm;
    }

    size_t get_width() const {
        return width;
    }

    size_t get_height() const {
        return height;
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

