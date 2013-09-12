/*
 * visibility_map.cpp
 *
 * Graph Library for Autonomous and Dynamic Systems
 *
 * author:  Pierrick Koch <pierrick.koch@laas.fr>
 * created: 2013-09-10
 * license: BSD
 */

#include "gladys/gdal.hpp"
#include "gladys/visibility_map.hpp"

namespace gladys {

void visibility_map::_load() {
    size_t width  = dtm.get_width();
    size_t height = dtm.get_height();
    // gdal::raster aka. vector<float>
    const auto& heightmap = dtm.get_band("Z_MAX");
    // TODO
}

} // namespace gladys
