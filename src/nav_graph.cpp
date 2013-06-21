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

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>

#include "gladys/nav_graph.hpp"

namespace gladys {

int nav_graph::load(const std::string filepath) {
    map.load(filepath);

    size_t band_id, px_x, px_y, size_x = map.get_x();
    vertex_t vert_w, vert_n, vert_e, vert_s;
    edge_t edge_ab, edge_bc, edge_cd, edge_da;
    double scale_x = map.get_scale_x(), scale_y = map.get_scale_y();
    float weight;
    raster weight_map = map.get_map();

    for (px_x = 0; px_x < map.get_x(); px_x++)
    for (px_y = 0; px_y < map.get_y(); px_y++) {

        weight = weight_map[px_x + px_y * size_x];

        if (is_obstacle(weight))
            continue; // do not create edge if obstacle

        vert_w = get_vertex(scale_x * (px_x - 0.5), scale_y * (px_y      ));
        vert_n = get_vertex(scale_x * (px_x      ), scale_y * (px_y - 0.5));
        vert_e = get_vertex(scale_x * (px_x + 0.5), scale_y * (px_y      ));
        vert_s = get_vertex(scale_x * (px_x      ), scale_y * (px_y + 0.5));

        // create edges and set weight
        // TODO length = math.sqrt( (.5*scale_x)**2 + (.5*scale_y)**2 )
        boost::add_edge(vert_w, vert_n, weight, g);
        boost::add_edge(vert_n, vert_e, weight, g);
        boost::add_edge(vert_e, vert_s, weight, g);
        boost::add_edge(vert_s, vert_w, weight, g);
        // also add straight connexions
        boost::add_edge(vert_n, vert_s, weight, g); // TODO length = scale_y
        boost::add_edge(vert_w, vert_e, weight, g); // TODO length = scale_x
    }
    return EXIT_SUCCESS;
}

} // namespace gladys


// TODO Boost.Test <utf> Unit Test Framework - http://www.boost.org/libs/test

int _test_point_to_vertex()
{
    gladys::nav_graph obj;
    obj.get_vertex(1,2);
    obj.get_vertex(2,3);
    obj.get_vertex(1,2);
    obj.get_vertex(3,2);
    obj.get_vertex(2,1);
    obj.get_vertex(2,3);
    obj.write_graphviz();
}

int _test_raster_to_graph()
{
    std::string path = "/tmp/gladys.tif";
    gladys::gdal tmp;
    tmp.set_size(5, 4, 4);
    tmp.save(path);
    gladys::nav_graph obj(path);
    obj.write_graphviz();

    return EXIT_SUCCESS;
}

int main(int argc, char * argv[])
{
    if (argc < 3)
        return 1;
    gladys::nav_graph obj(argv[1]);
    obj.save(argv[2]);

    _test_raster_to_graph();
    return EXIT_SUCCESS;
}
