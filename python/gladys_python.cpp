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

static bpy::list py_search(gladys::nav_graph& self, bpy::tuple start, bpy::tuple goal) {
    // optionally check that start and goal have the required
    // size of 2 using bpy::len()

    // convert arguments and call the C++ search method
    gladys::point_xy_t _start = {bpy::extract<double>(start[0]), bpy::extract<double>(start[1])};
    gladys::point_xy_t _goal  = {bpy::extract<double>(goal[0]), bpy::extract<double>(goal[1])};
    gladys::path_t cxx_retval = self.astar_search(_start, _goal);

    // converts the returned value into a list of 2-tuples
    bpy::list retval;
    for (auto &i : cxx_retval) retval.append(bpy::make_tuple(i[0], i[1]));
    return retval;
}

// Python requires an exported function called init<module-name> in every
// extension module. This is where we build the module contents.
BOOST_PYTHON_MODULE(libgladys_python)
{
    // gdal
    bpy::class_<gladys::gdal>("gdal", bpy::init<std::string>())
        // gdal::save
        .def("save", &gladys::gdal::save)
        ;
    // nav_graph
    bpy::class_<gladys::nav_graph>("nav_graph", bpy::init<std::string, std::string>())
        // nav_graph::save
        .def("save", &gladys::nav_graph::save)
        // nav_graph::astar_search
        .def("search", &py_search)
        ;
}
