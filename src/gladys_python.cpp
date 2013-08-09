/*
 * gladys_python.cpp
 *
 * Tool for Graph Library for Autonomous and Dynamic Systems
 *
 * author:  Pierrick Koch <pierrick.koch@laas.fr>
 * created: 2013-08-09
 * license: BSD
 */
#include <boost/python/class.hpp>
#include <boost/python/module.hpp>
#include <boost/python/def.hpp>
#include <string>

#include "gladys/gdal.hpp"
#include "gladys/nav_graph.hpp"

using namespace boost::python;

// Python requires an exported function called init<module-name> in every
// extension module. This is where we build the module contents.
BOOST_PYTHON_MODULE(libgladys_python)
{
    // gdal
    class_<gladys::gdal>("gdal", init<std::string>())
        // gdal::save
        .def("save", &gladys::gdal::save)
        ;
    // nav_graph
    class_<gladys::nav_graph>("nav_graph", init<std::string, std::string>())
        // nav_graph::save
        .def("save", &gladys::nav_graph::save)
        ;
}

/*
import libgladys_python
f_region = '/home/pkoch/sandbox/gladys/demo/maze.region.200.tif'
f_robot  = '/home/pkoch/sandbox/gladys/demo/robot.json'
g = libgladys_python.nav_graph(f_region, f_robot)
g.save('test.tif')
*/

