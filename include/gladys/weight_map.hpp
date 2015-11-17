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
#include <limits> // for numeric_limits::infinity
#include <map>

#include "gdalwrap/gdal.hpp"
#include "gladys/robot_model.hpp"

namespace gladys {

/*
 * from terrains model (in multi-layers GeoTiff)
 * to single layers weight map (after inflating obstacles by robot size)
 */
class weight_map {
protected:
    gdalwrap::gdal terrains; // probalistic models (multi-layers GeoTiff)
    gdalwrap::gdal map; // weight map (after inflating robot size)
    robot_model rmdl;
    size_t width ;
    enum {W_FLAG_OBSTACLE=-2, W_UNKNOWN=-1};
public:
    weight_map() {}
    weight_map(const gdalwrap::gdal& _map) {
        map = _map;
    }
    weight_map(const std::string& f_region, const std::string& f_robot_model) {
        load(f_region, f_robot_model);
    }
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
    virtual void _load();

    //TODO, this is not a merge !
    void merge(const weight_map& _wm) {
        map = _wm.get_map();
    }

    /**
     * NOTE: Don't forget to set_transform GeoData
     */
    gdalwrap::raster& setup_weight_band(size_t width, size_t height) {
        map.set_size(1, width, height);
        map.names[0] = "WEIGHT";
        return map.bands[0];
    }

    void flag_as_obstacle(float& weight) {
        if (!is_obstacle(weight))
            weight = W_FLAG_OBSTACLE;
    }
    bool is_flag_obstacle(float weight) const {
        return weight == W_FLAG_OBSTACLE;
    }
    bool is_obstacle(float weight) const {
        return weight == std::numeric_limits<float>::infinity();
    }

    const gdalwrap::raster& get_weight_band() const {
        return map.bands[0];
    }

    /** handy method to display the weight-map
     */
    std::vector<unsigned char> get_weight_band_uchar() const {
        const auto& weight = map.bands[0];
        std::vector<unsigned char> retval(weight.size());
        for (size_t idx = 0; idx < weight.size(); idx++) {
            const auto& val = weight[idx];
            if (val < 0)
                retval[idx] = 0;
            else if (val == std::numeric_limits<float>::infinity())
                retval[idx] = 255;
            else
                retval[idx] = std::floor(val * 5.0);
        }
        return retval;
    }

    const gdalwrap::gdal& get_map() const {
        return map;
    }
    gdalwrap::gdal& get_map() {
        return map;
    }
    const gdalwrap::gdal& get_region() const {
        return terrains;
    }
    const robot_model& get_robot() const {
        return rmdl;
    }

    size_t get_width() const {
        return map.get_width();
    }

    size_t get_height() const {
        return map.get_height();
    }

    double get_scale_x() const {
        return map.get_scale_x();
    }

    double get_scale_y() const {
        return map.get_scale_y();
    }

    double get_utm_pose_x() const {
        return map.get_utm_pose_x();
    }

    double get_utm_pose_y() const {
        return map.get_utm_pose_y();
    }

    size_t index( const point_xy_t& p ) const {
        return map.index_custom(p[0], p[1]);
    }

    size_t index_utm( const point_xy_t& p ) const {
        return map.index_utm(p[0], p[1]);
    }
    void save(const std::string& filepath) const {
        map.save(filepath);
    }
};

class costmap : public weight_map {
public:
    costmap(const std::string& f_region, const std::string& f_robot_model) {
        load(f_region, f_robot_model);
    }
    void _load();
};

} // namespace gladys

#endif // WEIGHT_MAP_HPP
