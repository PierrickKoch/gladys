/*
 * graph_astar.hpp
 *
 * Graph Library for Autonomous and Dynamic Systems
 *
 * author:  Pierrick Koch <pierrick.koch@laas.fr>
 * created: 2013-07-22
 * license: BSD
 */
#ifndef GRAPH_ASTAR_HPP
#define GRAPH_ASTAR_HPP

#include <cmath>
#include <array>
#include <vector>
#include <algorithm>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/astar_search.hpp>

#include "gladys/point.hpp"

namespace gladys {
// graph
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS,
    boost::property<boost::vertex_name_t, point_xy_t>,
    boost::property<boost::edge_weight_t, float> > graph_t;
typedef graph_t::vertex_descriptor vertex_t;
typedef graph_t::edge_descriptor edge_t;
typedef std::vector<vertex_t> vertices_t;
typedef std::map<point_xy_t, vertex_t> vertex_map_t;

typedef struct {
    path_t path;
    double cost; // TMP
    double utility; // TMP
} path_cost_util_t;

struct found_goal {
    vertex_t g;
    found_goal(vertex_t _g) : g(_g) {};
}; // exception for termination

// visitor that terminates when we find the goal
class astar_goal_visitor : public boost::default_astar_visitor {
    vertex_t goal;
public:
    astar_goal_visitor(vertex_t _goal) : goal(_goal) {}

    /** examine_vertex is invoked when a vertex is popped from the queue
     * (i.e., it has the lowest cost on the OPEN list).
     */
    void examine_vertex(vertex_t u, const graph_t& g) {
        // search if the current vertex is in the list of goal
        if (u == goal)
            throw found_goal(u);
    }
};

class astar_goals_visitor : public boost::default_astar_visitor {
    vertices_t goals;
public:
    astar_goals_visitor(vertices_t _goals) : goals(_goals) {}

    /** examine_vertex is invoked when a vertex is popped from the queue
     * (i.e., it has the lowest cost on the OPEN list).
     */
    void examine_vertex(vertex_t u, const graph_t& g) {
        // search if the current vertex is in the list of goal
        for (auto& v : goals)
            if (u == v)
                throw found_goal(u);
    }
};

/**
* Navigation heuristics functions for graph visitor algorithms
*/
class nav_goal_heuristic : public boost::astar_heuristic<graph_t, double> {
    const graph_t& g;
    const vertex_t& goal;
public:
    nav_goal_heuristic(const graph_t& _g, const vertex_t& _goal)
        : g(_g), goal(_goal) {}

    double operator()(const vertex_t& u) {
        const auto& vertex_point_map = boost::get(boost::vertex_name, g);
        return distance(vertex_point_map[u], vertex_point_map[goal]);
    }
};

/**
* Navigation heuristics functions for graph visitor algorithms
*/
class nav_goals_heuristic : public boost::astar_heuristic<graph_t, double> {
    const graph_t& g;
    const vertices_t& goals;
public:
    nav_goals_heuristic(const graph_t& _g, const vertices_t& _goals)
        : g(_g), goals(_goals) {}

    double operator()(const vertex_t& u) {
        // get closest distance to one of the goals
        const auto& vertex_point_map = boost::get(boost::vertex_name, g);
        auto first = goals.begin();
        double tmp, smallest = distance_sq(vertex_point_map[u], vertex_point_map[*first]);
        ++first;
        for (; first != goals.end(); ++first) {
            tmp = distance_sq(vertex_point_map[u], vertex_point_map[*first]);
            if (tmp < smallest) {
                smallest = tmp;
            }
        }
        return std::sqrt( smallest );
    }
};

} // namespace gladys

#endif // GRAPH_ASTAR_HPP

