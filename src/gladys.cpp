/*
 * Graph Library for Autonomous and Dynamic Systems
 */
#include <string>           // for string
#include <cassert>          // for assert
#include <vector>           // for vector

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>

#include "gladys/gladys.hpp"

namespace gladys {

using namespace std;
using namespace boost;

int gladys::init() {
    for (int band_id, idx = 0; idx < io.get_size(); idx++) {
        vertex v = add_vertex(g);
        edge e = add_edge(v1, v2, g);
        for (band_id = 0; band_id < io.bands.size(); band_id++) {
            // TODO arc[band_id] = band[idx];

        }
    }
    return 0;
}

} // namespace gladys

int main(int argc, char * argv[])
{
    if (argc < 3)
        return 1;
    gladys::gladys obj;
    obj.load(argv[1]);
    obj.init();
    obj.save(argv[2]);

    std::cout<<"gladys!"<<std::endl;
    return 0;
}

