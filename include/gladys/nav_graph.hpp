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

#include <cmath>
#include <array>
#include <deque>
#include <vector>
#include <string>
#include <fstream> // output file stream

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/depth_first_search.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/graphviz.hpp>

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
typedef std::deque<point_xy_t> path_t;

inline std::string to_string(const point_xy_t& value ) {
    return "[" + std::to_string(value[0]) + "," +
                 std::to_string(value[1]) + "]";
}
inline std::ostream& operator<<(std::ostream& os, const point_xy_t& value) {
    return os<<to_string(value);
}

inline std::string to_string(const point_xyz_t& value ) {
    return "[" + std::to_string(value[0]) + "," +
                 std::to_string(value[1]) + "," +
                 std::to_string(value[2]) + "]";
}
inline std::ostream& operator<<(std::ostream& os, const point_xyz_t& value) {
    return os<<to_string(value);
}

inline std::string to_string(const path_t& value ) {
    std::string arrow = "", buff = "";
    for (auto& elt : value) {
        buff += arrow + to_string(elt);
        arrow = " -> ";
    }
    return buff;
}
inline std::ostream& operator<<(std::ostream& os, const path_t& value) {
    return os<<to_string(value);
}


/** Euclidian distance (squared)
 * usefull to compare a set of points (faster)
 */
inline double distance_sq(const point_xy_t& pA, const point_xy_t& pB) {
    double x = pA[0] - pB[0];
    double y = pA[1] - pB[1];
    return x*x + y*y;
}
/** Euclidian distance */
inline double distance(const point_xy_t& pA, const point_xy_t& pB) {
    return std::sqrt(distance_sq(pA, pB));
}

struct found_goal {}; // exception for termination

// visitor that terminates when we find the goal
class astar_goal_visitor : public boost::default_astar_visitor {
    vertex_t goal;
public:
    astar_goal_visitor(vertex_t goal) : goal(goal) {}

    /** examine_vertex is invoked when a vertex is popped from the queue
     * (i.e., it has the lowest cost on the OPEN list).
     */
    void examine_vertex(vertex_t u, const graph_t& g) {
        if (u == goal)
            throw found_goal();
    }
};

/**
* Navigation heuristics functions for graph visitor algorithms
*/
class nav_heuristic : public boost::astar_heuristic<graph_t, double> {
    graph_t g;
    vertex_t goal;
public:
    nav_heuristic(const graph_t& _g, const vertex_t& _goal)
        : g(_g), goal(_goal) {}

    double operator()(const vertex_t& u) {
        const auto& vertex_point_map = boost::get(boost::vertex_name, g);
        return distance(vertex_point_map[u], vertex_point_map[goal]);
    }
};

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

    path_t astar_search(const point_xy_t& start, const point_xy_t& goal) {
        vertex_t goal_v = get_closest_vertex(goal);
        astar_goal_visitor vis(goal_v);
        path_t shortest_path;
        nav_heuristic heuristic(g, goal_v);
        std::vector<vertex_t> predecessors(num_vertices(g));
        std::vector<double> distances(boost::num_vertices(g));
        std::vector<double> ranks(boost::num_vertices(g), -1.0);
        std::vector<boost::default_color_type> colors(boost::num_vertices(g));
        try {
            boost::astar_search(
                g, get_closest_vertex(start), heuristic,
                boost::predecessor_map(predecessors.data()).
                    distance_map(distances.data()).
                    weight_map(boost::get(boost::edge_weight, g)).
                    rank_map(ranks.data()).
                    color_map(colors.data()).
                    visitor(vis)
            );
        } catch (found_goal) {
            const auto& vertex_point_map = boost::get(boost::vertex_name, g);
            for(vertex_t v = goal_v;; v = predecessors[v]) {
                shortest_path.push_front(vertex_point_map[v]);
                if (predecessors[v] == v)
                    break;
            }
        }
        return shortest_path;
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
    void write_graphviz(const std::string& filepath) const {
        std::ofstream of( filepath );
        write_graphviz( of );
        of.close();
    }

    int save(const std::string& filepath) {
        return map.save(filepath);
    }
};

inline std::ostream& operator<<(std::ostream& os, const nav_graph& ng) {
    ng.write_graphviz(os);
    return os;
}

} // namespace gladys

#endif // NAV_GRAPH_HPP

