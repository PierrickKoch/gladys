/*
 * nav_graph.hpp
 *
 * Graph Library for Autonomous and Dynamic Systems
 *
 * author:  Pierrick Koch <pierrick.koch@laas.fr>
 * created: 2013-06-12
 * license: BSD
 */
#ifndef NAV_GRAPH_HPP
#define NAV_GRAPH_HPP

#include <array>
#include <vector>
#include <string>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>

/* TODO? Boost.Geometry (since libboost1.47, Ubuntu 12.04 = libboost1.46)
include <boost/geometry/geometries/point_xy.hpp>
boost::geometry::model::d2::point_xy<double>
*/

#include "gladys/weight_map.hpp"

namespace gladys {

typedef std::array<double, 2> point_xy_t;  // XY
typedef std::array<double, 3> point_xyz_t; // XYZ
// graph
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS,
    boost::property<boost::vertex_name_t, point_xy_t>,
    boost::property<boost::edge_weight_t, float> > graph_t;
typedef boost::graph_traits<graph_t>::vertex_descriptor vertex_t;
typedef boost::graph_traits<graph_t>::edge_descriptor edge_t;
typedef std::map<point_xy_t, vertex_t> vertex_map_t;

std::string to_string( point_xy_t value ) {
    return std::to_string(value[0]) + "," +
           std::to_string(value[1]);
}

std::string to_string( point_xyz_t value ) {
    return std::to_string(value[0]) + "," +
           std::to_string(value[1]) + "," +
           std::to_string(value[2]);
}

/*
 * nav_graph
 */
class nav_graph {
    weight_map map;
    graph_t g;
    vertex_map_t vertices;
public:
    nav_graph() {}
    nav_graph(const std::string filepath) {
        load(filepath);
    }
    int load(const std::string filepath);

    // vertices
    vertex_t get_vertex(const double& x, const double &y) {
        point_xy_t p = {x, y};
        vertex_map_t::iterator it = vertices.find(p);
        if (it != vertices.end())
            return it->second;
        // else create new vertex setting p as property
        vertex_t v = boost::add_vertex(p, g);
        vertices[p] = v;
        return v;
    }

    bool is_obstacle(float weight) {
        return weight < 0;
    }

    /*
    // properties
    void set_vertex_property(const vertex_t& x, const point_xy_t& v) {
        boost::put(boost::vertex_name_t(), g, x, v);
    }
    */

    /** Write graphviz .dot file
     *
     * @param out output stream to write .dot data
     *
     * TIPS: use `display` [imagemagick] to visualize
     */
    void write_graphviz(std::ostream& out = std::cout) {
        std::vector<std::string> name(vertices.size());
        for (auto& kv : vertices)
            name[kv.second] = to_string(kv.first);
        boost::write_graphviz(out, g, boost::make_label_writer(name.data()));
    }

    int save(const std::string filepath) {
        return map.save(filepath);
    }
};

} // namespace gladys

#endif // NAV_GRAPH_HPP

