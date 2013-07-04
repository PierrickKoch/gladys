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
#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/depth_first_search.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/graphviz.hpp>

#include <boost/geometry/geometries/point_xy.hpp>
/* Boost.Geometry (since libboost1.47, Ubuntu 12.04 = libboost1.46) */

#include "gladys/weight_map.hpp"

namespace gladys {

typedef std::array<double, 2> point_xy_t;  // XY
typedef std::array<double, 3> point_xyz_t; // XYZ
// graph
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS,
    boost::property<boost::vertex_name_t, point_xy_t>,
    boost::property<boost::edge_weight_t, float> > graph_t;
typedef graph_t::vertex_descriptor vertex_t;
typedef graph_t::edge_descriptor edge_t;
typedef std::map<point_xy_t, vertex_t> vertex_map_t;

inline std::string to_string(const point_xy_t& value ) {
    return std::to_string(value[0]) + "," +
           std::to_string(value[1]);
}

inline std::string to_string(const point_xyz_t& value ) {
    return std::to_string(value[0]) + "," +
           std::to_string(value[1]) + "," +
           std::to_string(value[2]);
}

inline double distance_sq(const point_xy_t& pA, const point_xy_t& pB) {
    double x = pA[0] - pB[0];
    double y = pA[1] - pB[1];
    return x*x + y*y;
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
    /** nav_graph constructor
     *
     * @param f_region path to a region.tif file
     * (multi-layers terrains classification probabilities, float32)
     *
     * @param f_robot_model TODO path to a robot model
     * to generate the weight map (at least its size)
     *
     */
    nav_graph(const std::string& f_region, const std::string& f_robot_model) {
        load(f_region, f_robot_model);
    }
    int load(const std::string& f_region, const std::string& f_robot_model);

    // vertices
    vertex_t get_vertex(const double& x, const double &y) {
        point_xy_t p = {x, y};
        return get_vertex(p);
    }

    vertex_t get_vertex(const point_xy_t& p) {
        vertex_map_t::iterator it = vertices.find(p);
        if (it != vertices.end())
            return it->second;
        // else create new vertex setting p as property
        vertex_t v = boost::add_vertex(p, g);
        vertices[p] = v;
        return v;
    }

    vertex_t get_closest_vertex(const point_xy_t& p) {
        vertex_map_t::iterator it = vertices.find(p);
        if (it != vertices.end())
            return it->second;

        // else find closest vertex TODO optimize (scan all points)
        vertex_t closest_v;
        double tmp, closest_d = 1E+37;
        for (auto& kv : vertices) {
            tmp = distance_sq(p, kv.first);
            if (tmp < closest_d) {
                closest_v   = kv.second;
                closest_d = tmp;
            }
        }
        return closest_v;
    }

    void bfs(const point_xy_t& p) {
        boost::bfs_visitor<boost::null_visitor> vis; // TODO visitor get path
        boost::breadth_first_search(g, get_closest_vertex(p), boost::visitor(vis));
        // return TODO path from visitor
    }

    bool is_obstacle(float weight) {
        return weight < 0;
    }

    /** Write graphviz .dot file
     *
     * @param out output stream to write .dot data
     *
     * TIPS: use `neato -Tpng -Goverlap=false|display` to visualize
     *       or see in tools/dot_to_json.py
     */
    void write_graphviz(std::ostream& out = std::cout) const {
        std::vector<std::string> vert_label(vertices.size());
        for (auto& kv : vertices)
            vert_label[kv.second] = to_string(kv.first);
        // TODO write edge weight as well
        boost::write_graphviz( out, g,
            boost::make_label_writer(vert_label.data()) );
    }

    int save(const std::string& filepath) {
        return map.save(filepath);
    }
};

inline std::ostream& operator<<(std::ostream& os,
    const nav_graph& ng)
{
    ng.write_graphviz(os);
    return os;
}

} // namespace gladys

#endif // NAV_GRAPH_HPP

