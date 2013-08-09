/*
 * gladys_gui.cpp
 *
 * Tool for Graph Library for Autonomous and Dynamic Systems
 *
 * author:  Pierrick Koch <pierrick.koch@laas.fr>
 * created: 2013-07-31
 * license: BSD
 */
#include <string>
#include <iostream>
#include <thread>
#include <functional>

#include <QtGui>

#include "gladys/gdal.hpp"
#include "gladys/nav_graph.hpp"
#include "gladys/point.hpp"
#include "gladys/display.hpp"

std::string f1, f2;

void do_stuff(ImageViewer* image_viewer) {
    gladys::nav_graph ng(f1, f2);
    image_viewer->display(ng.get_map());
    // TODO thread
    //std::thread t1(graph_thread);

    gladys::display_hook_t func = std::bind(&ImageViewer::paint_point, image_viewer, std::placeholders::_1, std::placeholders::_2);
    ng.set_display_hook( func );

    gladys::point_xy_t p1 = {512, 1};
    gladys::point_xy_t p2 = {512, 512};//{(int)map.get_width(), (int)map.get_height()};
    //std::thread t1(std::bind(&gladys::nav_graph::astar_search_mono, ng, p1, p2));
    gladys::path_t path = ng.astar_search(p1, p2);
    //std::cout<<"path: "<<gladys::to_string(path)<<std::endl;
    for (const auto& p : path)
        image_viewer->paint(p, 3);

}

int main(int argc, char * argv[])
{
    if (argc < 3) {
        std::cerr<<"usage: gladys_gui region.tif robot.json"<<std::endl;
        return EXIT_FAILURE;
    }
    f1 = argv[1];
    f2 = argv[2];
    QtAppStart plop(argc, argv, do_stuff);
    // run
    return plop.exec();
}
