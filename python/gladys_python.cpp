/*
 * gladys_python.cpp
 *
 * Tool for Graph Library for Autonomous and Dynamic Systems
 *
 * author:  Pierrick Koch <pierrick.koch@laas.fr>
 * created: 2013-08-09
 * license: BSD
 */
#include <string>
#include <boost/python.hpp>

#include "gladys/gdal.hpp"
#include "gladys/nav_graph.hpp"
#include "gladys/frontier_exploration.hpp"

namespace bpy = boost::python;

template<class T>
bpy::list std_vector_to_py_list(const std::vector<T>& v) {
    bpy::list retval;
    for (const auto& elt : v)
        retval.append(elt);
    return retval;
}

static bpy::list py_search(gladys::nav_graph& self, bpy::tuple start, bpy::tuple goal) {
    // optionally check that start and goal have the required
    // size of 2 using bpy::len()

    // convert arguments and call the C++ search method
    gladys::point_xy_t _start = {bpy::extract<double>(start[0]), bpy::extract<double>(start[1])};
    gladys::point_xy_t _goal  = {bpy::extract<double>(goal[0]), bpy::extract<double>(goal[1])};
    gladys::path_t cxx_retval = self.astar_search(_start, _goal);

    // converts the returned value into a list of 2-tuples
    bpy::list retval;
    for (auto &i : cxx_retval)
        retval.append(bpy::make_tuple(i[0], i[1]));
    return retval;
}

static bpy::list py_compute_frontiers(gladys::frontier_detector& self, bpy::tuple seed) {
    // optionally check that seed has the required
    // size of 2 using bpy::len()

    // convert arguments and call the C++ search method
    gladys::point_xy_t _seed = {bpy::extract<double>(seed[0]), bpy::extract<double>(seed[1])};
    self.compute_frontiers(_seed);
    const std::vector< gladys::points_t > cxx_retval = self.get_frontiers();

    // converts the returned value into a list of lists of 2-tuples (= list of
    // frontiers)
    bpy::list retval;
    for (auto & _f : cxx_retval) {
        bpy::list f;
        for (auto &p : _f)
            f.append(bpy::make_tuple(p[0], p[1]));
        retval.append(f);
    }

    return retval;
}

static bpy::list py_get_band(gladys::gdal& self, const std::string& name) {
    return std_vector_to_py_list(self.get_band(name));
}

static bpy::list py_get_band_as_uchar(gladys::gdal& self, const std::string& name) {
    return std_vector_to_py_list(gladys::vfloat2vuchar(self.get_band(name)));
}

static bpy::dict py_get_bands(gladys::gdal& self) {
    bpy::dict retval;
    for (size_t idx = 0; idx < self.bands.size(); idx++)
        retval[ self.bands_name[idx] ] = std_vector_to_py_list( self.bands[idx] );
    return retval;
}

static bpy::dict py_get_bands_as_uchar(gladys::gdal& self) {
    bpy::dict retval;
    for (size_t idx = 0; idx < self.bands.size(); idx++)
        retval[ self.bands_name[idx] ] = std_vector_to_py_list(
            gladys::vfloat2vuchar( self.bands[idx] ) );
    return retval;
}

static gladys::weight_map py_nav_graph_get_map(gladys::nav_graph& self) {
    return self.get_map(); // cannot use reference with boost::python
}

static gladys::gdal py_weight_map_get_map(gladys::weight_map& self) {
    return self.get_map(); // cannot use reference with boost::python
}

static gladys::gdal py_weight_map_get_region(gladys::weight_map& self) {
    return self.get_region(); // cannot use reference with boost::python
}
static bpy::list py_weight_map_get_weight_band_uchar(gladys::weight_map& self) {
    return std_vector_to_py_list( self.get_weight_band_uchar() );
}

// Python requires an exported function called init<module-name> in every
// extension module. This is where we build the module contents.
BOOST_PYTHON_MODULE(libgladys_python)
{
    // gdal
    bpy::class_<gladys::gdal>("gdal", bpy::init<std::string>())
        .def("save", &gladys::gdal::save)
        .def("load", &gladys::gdal::load)
        .def("get_width", &gladys::gdal::get_width)
        .def("get_height", &gladys::gdal::get_height)
        .def("get_scale_x", &gladys::gdal::get_scale_x)
        .def("get_scale_y", &gladys::gdal::get_scale_y)
        .def("get_utm_pose_x", &gladys::gdal::get_utm_pose_x)
        .def("get_utm_pose_y", &gladys::gdal::get_utm_pose_y)
        // gdal::get_band
        .def("get_band", &py_get_band)
        // gdal::get_band distributed and converted from float to uchar [0-255]
        .def("get_band_as_uchar", &py_get_band_as_uchar)
        // gdal::bands
        .def("get_bands", &py_get_bands)
        // gdal::get_bands distributed and converted from float to uchar [0-255]
        .def("get_bands_as_uchar", &py_get_bands_as_uchar)
        ;
    // nav_graph
    bpy::class_<gladys::nav_graph>("nav_graph", bpy::init<std::string, std::string>())
        .def("get_map", &py_nav_graph_get_map)
        .def("save", &gladys::nav_graph::save)
        // nav_graph::astar_search
        .def("search", &py_search)
        ;
    // weight_map
    bpy::class_<gladys::weight_map>("weight_map", bpy::init<>())
        .def("save", &gladys::weight_map::save)
        .def("load", &gladys::weight_map::load)
        .def("get_width", &gladys::weight_map::get_width)
        .def("get_height", &gladys::weight_map::get_height)
        .def("get_scale_x", &gladys::weight_map::get_scale_x)
        .def("get_scale_y", &gladys::weight_map::get_scale_y)
        .def("get_utm_pose_x", &gladys::weight_map::get_utm_pose_x)
        .def("get_utm_pose_y", &gladys::weight_map::get_utm_pose_y)
        .def("get_map", &py_weight_map_get_map)
        .def("get_region", &py_weight_map_get_region)
        .def("get_weight_band_uchar", &py_weight_map_get_weight_band_uchar)
        ;
    // frontier_exploration
    bpy::class_<gladys::frontier_detector>("frontier_detector", bpy::init<std::string, std::string>())
        .def("compute_frontiers", &py_compute_frontiers)
        ;
}
