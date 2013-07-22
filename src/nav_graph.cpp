/*
 * nav_graph.cpp
 *
 * Graph Library for Autonomous and Dynamic Systems
 *
 * author:  Pierrick Koch <pierrick.koch@laas.fr>
 * created: 2013-06-12
 * license: BSD
 */
#include <string>
#include <cmath>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>

#include "gladys/graph_astar.hpp"
#include "gladys/nav_graph.hpp"

namespace gladys {

void nav_graph::_load() {
    size_t px_x, px_y, width = map.get_width();
    vertex_t vert_w, vert_n, vert_e, vert_s;
    double scale_x = map.get_scale_x(), scale_y = map.get_scale_y();
    float weight;
    raster weight_map = map.get_map();
    // most of the time this is equal to str(2)/2
    float hypotenuse = 0.5 * std::sqrt( scale_x*scale_x + scale_y*scale_y );

    for (px_x = 0; px_x < map.get_width(); px_x++)
    for (px_y = 0; px_y < map.get_height(); px_y++) {

        weight = weight_map[px_x + px_y * width];

        if (map.is_obstacle(weight))
            continue; // do not create edge if obstacle

        vert_w = get_vertex(scale_x * (px_x - 0.5), scale_y * (px_y      ));
        vert_n = get_vertex(scale_x * (px_x      ), scale_y * (px_y - 0.5));
        vert_e = get_vertex(scale_x * (px_x + 0.5), scale_y * (px_y      ));
        vert_s = get_vertex(scale_x * (px_x      ), scale_y * (px_y + 0.5));

        // create edges and set weight
        // length = .5 * math.sqrt( scale_x**2 + scale_y**2 )
        boost::add_edge(vert_w, vert_n, hypotenuse * weight, g);
        boost::add_edge(vert_n, vert_e, hypotenuse * weight, g);
        boost::add_edge(vert_e, vert_s, hypotenuse * weight, g);
        boost::add_edge(vert_s, vert_w, hypotenuse * weight, g);
        // also add straight connexions
        boost::add_edge(vert_n, vert_s, scale_y * weight, g); // length = scale_y
        boost::add_edge(vert_w, vert_e, scale_x * weight, g); // length = scale_x
    }
}

path_t nav_graph::astar_search(const point_xy_t& start, const point_xy_t& goal) {
    vertex_t goal_v = get_closest_vertex(goal);
    astar_goal_visitor vis(goal_v);
    path_t shortest_path;
    nav_goal_heuristic heuristic(g, goal_v);
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

path_cost_util_t nav_graph::astar_search(const points_t& start, const points_t& goal) {
    vertices_t goal_v;
    for (auto& p : goal)
        goal_v.push_back( get_closest_vertex( p ) );

    astar_goals_visitor vis(goal_v);

    path_t shortest_path;
    nav_goals_heuristic heuristic(g, goal_v);
    std::vector<vertex_t> predecessors(num_vertices(g));
    std::vector<double> distances(boost::num_vertices(g));
    std::vector<double> ranks(boost::num_vertices(g), -1.0);
    std::vector<boost::default_color_type> colors(boost::num_vertices(g));
    try {
        // NOTE: pass by a "virtual node" as a starting point in the OPEN list (see: color map)
        boost::astar_search(
            g, get_closest_vertex(start[0]), heuristic,
            boost::predecessor_map(predecessors.data()).
                distance_map(distances.data()).
                weight_map(boost::get(boost::edge_weight, g)).
                rank_map(ranks.data()).
                color_map(colors.data()).
                visitor(vis)
        );
    } catch (found_goal& e) {
        const auto& vertex_point_map = boost::get(boost::vertex_name, g);
        for(vertex_t v = e.g;; v = predecessors[v]) {
            shortest_path.push_front(vertex_point_map[v]);
            if (predecessors[v] == v)
                break;
        }
    }
    path_cost_util_t res;
    res.path = shortest_path;
    return res;
}

} // namespace gladys
