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

#include <vector>
#include <string>
#include <fstream> // output file stream

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/graphviz.hpp>

#include "gladys/point.hpp"
#include "gladys/graph_astar.hpp"
#include "gladys/weight_map.hpp"

namespace gladys {

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
    void load(const std::string& f_region, const std::string& f_robot_model) {
        map.load(f_region, f_robot_model);
        _load();
    }
    void _load();

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
                closest_v = kv.second;
                closest_d = tmp;
            }
        }
        return closest_v;
    }

    path_t astar_search(const point_xy_t& start, const point_xy_t& goal);
    path_cost_util_t astar_search(const points_t& start, const points_t& goal);

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
    void write_graphviz(const std::string& filepath) const {
        std::ofstream of( filepath );
        write_graphviz( of );
        of.close();
    }

    void save(const std::string& filepath) {
        map.save(filepath);
    }
};

inline std::ostream& operator<<(std::ostream& os, const nav_graph& ng) {
    ng.write_graphviz(os);
    return os;
}

} // namespace gladys

#endif // NAV_GRAPH_HPP

