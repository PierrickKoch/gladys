/*
 * Graph Library for Autonomous and Dynamic Systems
 */
#include <iostream>         // for string
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <gdal_priv.h>

using namespace std;
using namespace boost;

int main(int argc, char * argv[])
{
    GDALDataset  *dataset;
    if (argc < 2)
        return 1;
    GDALAllRegister();
    dataset = (GDALDataset *) GDALOpen( argv[1], GA_ReadOnly );
    if (!dataset)
        return 1;
    // create a typedef for the Graph type
    typedef adjacency_list<vecS, vecS, bidirectionalS> Graph;
    // declare a graph object
    Graph g();

    //

    std::cout<<"gladys!"<<std::endl;
    return 0;
}

