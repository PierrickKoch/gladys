/*
 * dstar.hpp
 *
 * Graph Library for Autonomous and Dynamic Systems
 *
 * author:  Arnaud Degroote <arnaud.degroote@laas.fr>
 * created: 2013-09-10
 * license: BSD
 */

#ifndef DSTAR_HPP
#define DSTAR_HPP 

#include <boost/bimap/bimap.hpp>
#include <boost/bimap/unordered_set_of.hpp>
#include <boost/bimap/multiset_of.hpp>

#include "gladys/graph_astar.hpp"

namespace gladys {

    class no_path {};

    class dstar_search {
        private:

            typedef std::pair<float, float> key_t;

            typedef boost::bimaps::bimap <
                boost::bimaps::unordered_set_of<vertex_t>,
                boost::bimaps::multiset_of<key_t, std::less<key_t>>
            > bm_type;

            typedef bm_type::value_type bm_value;

            struct cost {
                float g;
                float rhs;
            };

            const graph_t& g;
            vertex_t start, goal, last;
            time_t t;
            bm_type pq;
            std::vector<cost> costs;
            float km;

            key_t calc_key(const vertex_t& v) const;

            float h(const vertex_t& v1, const vertex_t& v2) const;

            void update_vertex(const vertex_t& v);
            void compute_shortest_path();
            
            void print_pq() const;


            struct graph_writer { 
                void operator() (std::ostream& os) const;
            };

        public:
            dstar_search(const graph_t& g, const vertex_t& start, const vertex_t& goal);

            void write_graphviz(std::ostream& os) const;

            path_t get_path() const;

            void replan(const vertex_t& now);

    };


} // namespace gladys

#endif // DSTAR_HPP

