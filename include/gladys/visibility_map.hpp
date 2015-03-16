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

#include "gdalwrap/gdal.hpp"
#include "gladys/robot_model.hpp"
#include "gladys/point.hpp"

namespace gladys {

/*
 * from digital terrain map (in multi-layers GeoTiff)
 */
class visibility_map {
    gdalwrap::gdal dtm; // digital terrain map (multi-layers GeoTiff)
    robot_model rmdl;
    size_t width;  // dtm width
    size_t height; // dtm height

    void _load();

public:
    visibility_map() {}
    visibility_map(const std::string& f_dtm, const std::string& f_robot_model) {
        load(f_dtm, f_robot_model);
    }
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
     * Assume an height of 0 for the target and 0 for the sensor
     *
     * @param s the position of the sensor
     *
     * @param t the position of the target
     *
     * @returns true if visible.
     *
     */
    bool is_visible( const point_xy_t& s, const point_xy_t& t) const ;

    /** test if point 't' (target) is visible from 's' (sensor)
     *
     * @param s the position of the sensor
     *
     * @param t the position of the target
     *
     * @returns true if visible.
     *
     */
    bool is_visible( const point_xyz_t& s, const point_xyz_t& t) const ;

    /** test if point 't' (target) is visible from 's' (sensor)
     * Assume an height of 0 for the target and use the sensor pose for the sensor height.
     * Also use the sensor range
     *
     * @param s the position of the sensor
     *
     * @param t the position of the target
     *
     * @returns true if visible.
     *
     */
    bool is_sensor_visible( const point_xy_t& s, const point_xy_t& t) const ;

    /** test if point 't' (target) is visible from 's' (sensor)
     * Assume an height of 0 for the target and use the sensor pose for the sensor height.
     * Also use the sensor range
     *
     * @param s the position of the sensor
     *
     * @param t the position of the target
     *
     * @returns true if visible.
     *
     */
    bool is_sensor_visible( const point_xyz_t& s, const point_xyz_t& t) const ;

    /** test if point 't' (target) is visible from 'a' (antenna)
     * Assume an height of 0 for the target and use the antenna pose for the antenna height
     * Also use the antenna range
     *
     * @param s the position of the sensor
     *
     * @param t the position of the target
     *
     * @returns true if visible.
     *
     */
    bool is_antenna_visible( const point_xy_t& a, const point_xy_t& t) const ;

    /** test if point 't' (target) is visible from 'a' (antenna)
     * Assume an height of 0 for the target and use the antenna pose for the antenna height
     * Also use the antenna range
     *
     * @param s the position of the sensor
     *
     * @param t the position of the target
     *
     * @returns true if visible.
     *
     */
    bool is_antenna_visible( const point_xyz_t& a, const point_xyz_t& t) const ;

    /** Get the index of the point in the raster
     *
     * @param p the point
     *
     * @returns the index of the point
     *
     */
    size_t index( const point_xy_t& p ) const {
        return dtm.index_custom(p[0], p[1]);
    }

    /* getters */
    const gdalwrap::raster& get_heightmap() const {
        return dtm.get_band("Z_MAX");
    }

    const gdalwrap::raster& get_npointsmap() const {
        return dtm.get_band("N_POINTS");
    }

    const gdalwrap::gdal& get_dtm() const {
        return dtm;
    }

    size_t get_width() const {
        return width;
    }

    size_t get_height() const {
        return height;
    }

    void save(const std::string& filepath) {
        dtm.save(filepath);
    }
};

} // namespace gladys

#endif // VISIBILITY_MAP_HPP

