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
#include <sstream> // ostringstream

#include "gladys/point.hpp"
#include "gladys/graph_astar.hpp"
#include "gladys/weight_map.hpp"

namespace gladys {

// Thanks to http://stackoverflow.com/questions/13453350
template<class It>
boost::iterator_range<It> pair_range(std::pair<It, It> const& p){
  return boost::make_iterator_range(p.first, p.second);
}

/*
 * nav_graph
 */
class nav_graph {
    weight_map map;
    graph_t g;
    vertex_map_t vertices_graphviz;
    std::vector<vertex_t> vv;
    size_t width;
    size_t height;
    double scale_x;
    double scale_y;

    vertex_t get_vertex_or_create(double x, double y) {
        vertex_t v = in_graph(x, y);
        if (is_valid(v))
            return v;
        // else create new vertex setting p as property
        return new_vertex(x, y);
    }
    vertex_t get_vertex_or_create(const point_xy_t& p) {
        vertex_t v = in_graph(vv_idx(p));
        if (is_valid(v))
            return v;
        // else create new vertex setting p as property
        return new_vertex(p);
    }

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
        width   = map.get_width();
        height  = map.get_height();
        scale_x = map.get_scale_x();
        scale_y = map.get_scale_y();
        // vv.assign(_vv_idx_no_check((1+height) * scale_x, (1+width) * scale_y), -1);
        std::cout<<"map loaded"<<std::endl;
        vv.assign(_vv_idx_no_check(height * scale_x, width * scale_y), -1);
        _load();
    }
    void _load();

    // vertices
    vertex_t new_vertex(const point_xy_t& p) {
        vertex_t v = boost::add_vertex(g);
        g[v].pt = p;
        vertices_graphviz[p] = v; // for write_graphviz
        add_vv(vv_idx(p), v);
        return v;
    }
    vertex_t new_vertex(double x, double y) {
        point_xy_t p = {x, y};
        vertex_t v = boost::add_vertex(g);
        g[v].pt = p;
        vertices_graphviz[p] = v; // for write_graphviz
        add_vv(vv_idx(x, y), v);
        return v;
    }

    size_t vv_idx(const point_xy_t& p) {
        return vv_idx(p[0], p[1]);
    }
    size_t _vv_idx_no_check(double x, double y) {
        return std::lround(4 * (x/scale_x + 0.5 + ((y/scale_y + 0.5) * width)));
    }
    size_t vv_idx(double x, double y) {
        size_t idx = _vv_idx_no_check(x, y);
        if (idx < 0 or idx > vv.size()) {
            std::ostringstream oss;
            oss<<"[nav_graph::vv_idx] "<<idx<<" ["<<x<<", "<<y<<"] out of range";
            throw std::out_of_range(oss.str());
        }
        return idx;
    }
    vertex_t in_graph(double x, double y) {
        return in_graph(vv_idx(x, y));
    }
    vertex_t in_graph(size_t idx) {
        return vv[idx];
    }
    void add_vv(const point_xy_t& p, vertex_t v) {
        add_vv(vv_idx(p), v);
    }
    void add_vv(size_t idx, vertex_t v) {
        if (idx < 0 or idx > vv.size())
            throw std::out_of_range("[nav_graph::add_vv] WTF ???");
        vv[idx] = v;
    }
    vertex_t get_closest_vertex(const point_xy_t& p) {
        return get_closest_vertex(p[0], p[1]);
    }
    bool is_valid(vertex_t v) {
        return v >= 0;
    }
    vertex_t get_closest_vertex(double x, double y) {
        vertex_t v = in_graph(x, y);
        if (is_valid(v))
            return v;
        // else try around TODO that's ugly
        v = in_graph(x - 0.5, y);
        if (is_valid(v))
            return v;
        v = in_graph(x + 0.5, y);
        if (is_valid(v))
            return v;
        v = in_graph(x, y - 0.5);
        if (is_valid(v))
            return v;
        v = in_graph(x, y + 0.5);
        if (is_valid(v))
            return v;
        v = in_graph(x - 0.5, y - 0.5);
        if (is_valid(v))
            return v;
        v = in_graph(x - 0.5, y + 0.5);
        if (is_valid(v))
            return v;
        v = in_graph(x + 0.5, y - 0.5);
        if (is_valid(v))
            return v;
        v = in_graph(x + 0.5, y + 0.5);
        if (is_valid(v))
            return v;
        // else nothing around
        return -1;
    }

    point_xy_t get_closest_navigable_point(double x, double y) {
        point_xy_t not_found = {0, 0};
        // get the closest vertex in the graph from the point {x, y}
        vertex_t vert = get_closest_vertex(x, y);
        if (!is_valid(vert))
            return not_found;
        boost::graph_traits<graph_t>::out_edge_iterator ei,ei_end;
        // for each edges of this vertex
        for ( const auto& edg : pair_range(boost::out_edges(vert, g)) )
        //for (boost::tie(ei,ei_end) = boost::out_edges(vert, g); ei != ei_end; ++ei)
            // if the edge is not an obstacle
            //if (g[*ei].weight < HUGE_VALF)
            if (g[edg].weight < HUGE_VALF)
                // return the point of this vertex
                return g[vert].pt;
        // if here, it means that the point is in an obstacle
        return not_found;
    }

    point_xy_t get_closest_navigable_point(double x, double y, double range) {
        point_xy_t tmp, not_found = {0, 0};
        tmp = get_closest_navigable_point(x, y);
        if (tmp != not_found)
            return tmp;
        // TODO that's ugly.
        for (double i = 0.5; i < range / 2; i += 0.5) {
            tmp = get_closest_navigable_point(x + i, y    );
            if (tmp != not_found)
                return tmp;
            tmp = get_closest_navigable_point(x    , y + i);
            if (tmp != not_found)
                return tmp;
            tmp = get_closest_navigable_point(x + i, y + i);
            if (tmp != not_found)
                return tmp;
            tmp = get_closest_navigable_point(x - i, y    );
            if (tmp != not_found)
                return tmp;
            tmp = get_closest_navigable_point(x    , y - i);
            if (tmp != not_found)
                return tmp;
            tmp = get_closest_navigable_point(x - i, y - i);
            if (tmp != not_found)
                return tmp;
        }
        return not_found;
    }

    points_t get_areas() {
        return get_areas(map.get_robot_model().get_sensor_range() / 2);
    }
    points_t get_areas(double range) {
        points_t areas;
        point_xy_t area, not_found = {0, 0};
        // get points in the graph which are not OBSTACLE every 2*range
        size_t sr = std::ceil(range + 1);
        size_t mx = map.get_width()  / map.get_scale_x() - sr;
        size_t my = map.get_height() / map.get_scale_y() - sr;
        for (size_t ix = sr; ix < mx; ix += sr)
        for (size_t iy = sr; iy < my; iy += sr) {
            area = get_closest_navigable_point(ix, iy, range);
            if (area != not_found)
                areas.push_back(area);
            //else
            //    std::cerr<<"area "<<area<<" is not found in ["<<ix<<", "<<iy<<"]"<<std::endl;
        }
        return areas;
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
    void write_graphviz(std::ostream& out = std::cout) const;
    void write_graphviz(const std::string& filepath) const;

    void save(const std::string& filepath) {
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

