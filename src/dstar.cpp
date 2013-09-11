/*
 * nav_graph.cpp
 *
 * Graph Library for Autonomous and Dynamic Systems
 *
 * author:  Arnaud Degroote <arnaud.degroote@laas.fr>
 * created: 2013-09-10
 * license: BSD
 */
#include "gladys/dstar.hpp"
#include <boost/graph/graphviz.hpp>
#include <limits>
#include <iostream>


namespace gladys {

    dstar_search::dstar_search(const graph_t& g, const vertex_t& start, 
                                                 const vertex_t& goal):
        g(g), start(start), goal(goal), last(start), t(std::time(0)),
        costs(boost::num_vertices(g), 
              {std::numeric_limits<float>::infinity(),
               std::numeric_limits<float>::infinity()}
              ), km(0)
    {
        costs[goal].rhs = 0;
        pq.insert(bm_value(goal, calc_key(goal)));
        compute_shortest_path();
    }

    dstar_search::key_t 
    dstar_search::calc_key(const vertex_t& v) const
    {
        const cost& c = costs[v];
        float min = std::min(c.rhs, c.g);
        return std::make_pair(min + h(start, v) + km, min);
    }

    void 
    dstar_search::update_vertex(const vertex_t& v)
    {
        cost& c = costs[v];
        if (v != goal) {
            float min = std::numeric_limits<float>::infinity();
            boost::graph_traits<graph_t>::out_edge_iterator ei,ei_end;
            for (boost::tie(ei,ei_end) = boost::out_edges(v, g); ei != ei_end; ++ei) 
                min = std::min(min, g[*ei].weight + costs[target(*ei, g)].g);
            c.rhs = min;
        }

        auto it = pq.left.find(v);
        if (it != pq.left.end()) pq.left.erase(it);

        if (c.g != c.rhs) 
            pq.insert(bm_value(v, calc_key(v)));
    }

    void
    dstar_search::compute_shortest_path() 
    {
        int i = 0;
        while (pq.right.begin()->first < calc_key(start) or 
               costs[start].rhs != costs[start].g)
        {
            auto it = pq.right.begin();
            key_t kold = it->first;
            vertex_t v = it->second;
            pq.right.erase(it);

            key_t k = calc_key(v);
            cost& cv = costs[v];

            if (kold < k) {
                pq.insert(bm_value(v, k));
            } else if (cv.g > cv.rhs) {
                cv.g = cv.rhs;
                boost::graph_traits<graph_t>::adjacency_iterator vi,vi_end;
                for (boost::tie(vi, vi_end) = boost::adjacent_vertices(v, g);
                    vi != vi_end; ++vi) {
                    update_vertex(*vi);
                }
            } else {
                cv.g = std::numeric_limits<float>::infinity();
                update_vertex(v);
                boost::graph_traits<graph_t>::adjacency_iterator vi,vi_end;
                for (boost::tie(vi, vi_end) = boost::adjacent_vertices(v, g);
                    vi != vi_end; ++vi)
                    update_vertex(*vi);
            }

            ++i;
        }

        if (pq.empty()) throw no_path();
    }
    
    float
    dstar_search::h(const vertex_t& v1, const vertex_t& v2) const
    {
        return distance(g[v1].pt, g[v2].pt);
    }
            
    
    path_t 
    dstar_search::get_path() const
    {
        path_t res;
        if (costs[last].g == std::numeric_limits<float>::infinity())
            throw no_path();

        res.push_back(g[last].pt);

        vertex_t v = last;
        while (v != goal) {
            boost::graph_traits<graph_t>::out_edge_iterator ei,ei_end;
            boost::tie(ei, ei_end) = boost::out_edges(v, g);
            vertex_t best_v = boost::target(*ei, g);
            float min = g[*ei].weight + costs[best_v].g;
            ++ ei;
            for (; ei != ei_end; ++ei) {
                vertex_t v1 = boost::target(*ei, g);
                float c = g[*ei].weight + costs[v1].g;
                if (min > c) {
                    min = c;
                    best_v = v1;
                }
            }
            v = best_v;
            res.push_back(g[v].pt);
        }

        return res;
    }

    void
    dstar_search::print_pq() const
    {
        for( bm_type::right_const_iterator
            i = pq.right.begin(), iend = pq.right.end();
            i != iend ; ++i )
        {
            std::cerr << i->second << " with [" << i->first.first;
            std::cerr << ", " << i->first.second << "]" << std::endl;
        }
    }

    struct graph_property_writer 
    {
        void operator()(std::ostream& out) const
        {
            out << "node [shape=box]" << std::endl;
        }
    };
    
    void
    dstar_search::write_graphviz(std::ostream& os) const
    {
        std::vector<std::string> vert_label(costs.size());
        boost::graph_traits<graph_t>::vertex_iterator vi,vi_end;
        boost::tie(vi, vi_end) = boost::vertices(g);
        for (; vi != vi_end; ++vi) {
            std::ostringstream oss;
            const cost& c = costs[*vi];
            oss << "pt " << g[*vi].pt << " g: " << c.g << " rhs: " << c.rhs;
            vert_label[*vi] = oss.str();
        }

        boost::write_graphviz(os, g, boost::make_label_writer(vert_label.data()),
                                     boost::default_writer(),
                                     graph_property_writer());
    }

    void
    dstar_search::replan(const vertex_t& v) 
    {
        time_t current_time = std::time(0);
        km = km + h(v, last);
        last = v;

        boost::graph_traits<graph_t>::edge_iterator ei, ei_end;
        for (boost::tie(ei, ei_end) = boost::edges(g) ; ei != ei_end; ++ei)
            if (g[*ei].t > t) {
                vertex_t source = boost::source(*ei, g);
                vertex_t target = boost::target(*ei, g);
                update_vertex(source);
                update_vertex(target);
            }

        t = current_time;

        compute_shortest_path();
    }
} // namespace gladys
