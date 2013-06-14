/*
 * gladys.hpp
 *
 * Graph Library for Autonomous and Dynamic Systems
 *
 * author:  Pierrick Koch <pierrick.koch@laas.fr>
 * created: 2013-06-12
 * license: BSD
 */
#ifndef GLADYS_HPP
#define GLADYS_HPP

#include <vector>
#include <boost/graph/adjacency_list.hpp>

#include "gladys/gdal.hpp"

namespace gladys {
    typedef boost::adjacency_list<boost::listS, boost::vecS,
        boost::bidirectionalS> graph;
    typedef boost::graph_traits<graph>::vertex_descriptor vertex;
    typedef boost::graph_traits<graph>::edge_descriptor edge;

    /*
     * gladys
     */
    class gladys {
        graph g;
        gdal io;
    public:
        int init();
        int load(const std::string filepath) {
            return io.load(filepath);
        }
        int save(const std::string filepath) {
            return io.save(filepath);
        }
    };

}

#endif // GLADYS_HPP

