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

#include <string>
#include <fstream> // output file stream

#include "gladys/point.hpp"
#include "gladys/graph_astar.hpp"
#include "gladys/weight_map.hpp"

namespace gladys {

/*
 * nav_graph
 */
class nav_graph {
    const weight_map& map;
    graph_t g;
    vertex_map_t vertices;
    size_t width;
    size_t height;
    double scale_x;
    double scale_y;

    vertex_t get_vertex_or_create(double x, double y) {
        point_xy_t p = {x, y};
        return get_vertex_or_create(p);
    }
    vertex_t get_vertex_or_create(const point_xy_t& p) {
        vertex_map_t::iterator it = vertices.find(p);
        if (it != vertices.end())
            return it->second;
        // else create new vertex setting p as property
        return new_vertex(p);
    }

public:
    /** nav_graph constructor
     *
     * @param w_map single layers weight map
     */
    nav_graph(const weight_map& w_map) : map(w_map) {
        width   = map.get_width();
        height  = map.get_height();
        scale_x = map.get_scale_x();
        scale_y = map.get_scale_y();
        _load();
    }
    void _load();

    // vertices
    vertex_t new_vertex(const point_xy_t& p) {
        vertex_t v = boost::add_vertex(g);
        g[v].pt = p;
        vertices[p] = v;
        return v;
    }
    vertex_t new_vertex(double x, double y) {
        point_xy_t p = {x, y};
        return new_vertex(p);
    }

    vertex_t get_closest_vertex(const point_xy_t& p) const {
        vertex_map_t::const_iterator it = vertices.find(p);
        if (it != vertices.end())
            return it->second;

        // else find closest vertex TODO optimize (scan all points)
        vertex_t closest_v;
        double tmp, closest_d = 1E+37;
        for (const auto& kv : vertices) {
            tmp = distance_sq(p, kv.first);
            if (tmp < closest_d) {
                closest_v = kv.second;
                closest_d = tmp;
            }
        }
        return closest_v;
    }

    path_t astar_search(const point_xy_t& start, const point_xy_t& goal) const;
    path_cost_util_t astar_search(const points_t& start, const points_t& goal) const;
    path_cost_util_t astar_search_custom(const points_t& start,
                                         const points_t& goal) const {
        // from custom frame to UTM and back to custom
        path_cost_util_t pcu = astar_search( custom_to_utm(start),
                                             custom_to_utm(goal ) );
        pcu.path = utm_to_custom(pcu.path);
        return pcu;
    }

    point_xy_t custom_to_utm(const point_xy_t& p) const {
        return map.get_map().point_custom2utm(p[0], p[1]);
    }
    point_xy_t utm_to_custom(const point_xy_t& p) const {
        return map.get_map().point_utm2custom(p[0], p[1]);
    }
    points_t custom_to_utm(const points_t& pts) const {
        points_t ptsc;
        for (const auto& p : pts)
            ptsc.push_back( custom_to_utm(p) );
        return ptsc;
    }
    path_t utm_to_custom(const path_t& pts) const {
        path_t ptsc;
        for (const auto& p : pts)
            ptsc.push_back( utm_to_custom(p) );
        return ptsc;
    }

    /** Write graphviz .dot file
     *
     * @param out output stream to write .dot data
     *
     * TIPS: use `neato -Tpng -Goverlap=false|display` to visualize
     *       or see in tools/dot_to_json.py
     */
    void write_graphviz(std::ostream& out = std::cout) const;
    void write_graphviz(const std::string& filepath) const;

    void save(const std::string& filepath) const {
        map.save(filepath);
    }
    const weight_map& get_map() const {
        return map;
    }

   const graph_t& get_graph() const {
       return g;
   }
};

inline std::ostream& operator<<(std::ostream& os, const nav_graph& ng) {
    ng.write_graphviz(os);
    return os;
}

} // namespace gladys

#endif // NAV_GRAPH_HPP

