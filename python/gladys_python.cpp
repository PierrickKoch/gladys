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

static bpy::list py_get_band(gladys::gdal& self, const std::string& name) {
    return std_vector_to_py_list(self.get_band(name));
}

static bpy::dict py_get_bands(gladys::gdal& self) {
    bpy::dict retval;
    for (size_t idx = 0; idx < self.bands.size(); idx++)
        retval[ self.bands_name[idx] ] = std_vector_to_py_list( self.bands[idx] );
    return retval;
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
        // gdal::bands
        .def("get_bands", &py_get_bands)
        ;
    // nav_graph
    bpy::class_<gladys::nav_graph>("nav_graph", bpy::init<std::string, std::string>())
        .def("get_map", &gladys::nav_graph::get_map)
        .def("save", &gladys::nav_graph::save)
        // nav_graph::astar_search
        .def("search", &py_search)
        ;
    // weight_map
    bpy::class_<gladys::weight_map>("weight_map", bpy::init<>())
        .def("get_map", &gladys::weight_map::get_map)
        .def("get_region", &gladys::weight_map::get_region)
        ;
}
