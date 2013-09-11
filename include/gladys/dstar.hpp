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
        /*
         * This is a fairly trivial implementation of dstar lite as described
         * in 'Fast replanning for navigation in unknown terrain' page 5 (fig
         * 5).
         *
         * The only non-trivial point is the use of boost::bimap to implement
         * the priority_queue, as one need to:
         *   - fastly get the smallest key
         *   - fastly find and erase an element by its vertex identifier.
         * In particular, use of std::priority_queue does not match the second
         * prerequisite.
         *
         * As possible, I keep the same variable name than the one used in the
         * algorithm description.
         *
         * Note that currently, the considered graph is not oriented, so
         * succ(v) == prev(v)
         *
         * TODO: allow to pass an user-defined heuristic. h is for moment the
         * euclidean distance between two vertices.
         * XXX: the replan part is not well tested, as the API does not allow
         * to upgrade a graph.
         */
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

        public:
            dstar_search(const graph_t& g, const vertex_t& start, const vertex_t& goal);

            void write_graphviz(std::ostream& os) const;

            path_t get_path() const;

            void replan(const vertex_t& now);
    };


} // namespace gladys

#endif // DSTAR_HPP

