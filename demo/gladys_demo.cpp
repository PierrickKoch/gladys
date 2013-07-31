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
#include <functional>

#include <QtGui>

#include "gladys/gdal.hpp"
#include "gladys/nav_graph.hpp"
#include "gladys/point.hpp"
// https://qt-project.org/doc/qt-4.8/widgets-imageviewer.html
// http://qt-project.org/doc/qt-4.8/qcolor.html#setHsv
typedef unsigned char uchar;
typedef std::vector<uchar> image8u_t;
image8u_t get_image8u(const gladys::gdal& in, size_t band_id=0) {
    const auto& band = in.bands[band_id];
    auto minmax = std::minmax_element(band.begin(), band.end());
    float min = *minmax.first;
    float max = *minmax.second;
    float diff = max - min;
    std::vector<uchar> img(band.size());
    if (diff == 0) {
        // max == min (useless band)
        std::cerr<<"[wran] get_mono: min == max"<<std::endl;
        return img;
    }
    float hue;
    for (size_t pose = 0; pose < img.size(); pose++) {
        hue = (band[pose] - min) / diff;
        if (hue > 1) hue = 1;
        img[pose] = hue * 255;
    }
    return img;
}

class ImageViewer : public QMainWindow {
    QLabel *_image_label;
    QImage _image;
public:
    ImageViewer() {
        setWindowTitle(tr("Image Viewer"));
        _image_label = new QLabel;
        setCentralWidget(_image_label);
    }
    void display(const QPixmap& pixmap) {
        _image_label->setPixmap(pixmap);
    }
    void display(const QImage& image) {
        _image = image;
        display(QPixmap::fromImage(image));
    }
    void display(const std::string& filepath) {
        QImage image(filepath.c_str());
        display(image);
    }
    void display(const gladys::gdal& in) {
        image8u_t image = get_image8u(in);
        QImage qimg(image.data(), in.get_width(), in.get_height(), QImage::Format_Indexed8);
        // Drawing into a QImage with QImage::Format_Indexed8 is not supported.
        QImage image_rgb = qimg.convertToFormat(QImage::Format_ARGB32);
        display(image_rgb);
    }
    void display_gdal(const std::string& filepath) {
        display(gladys::gdal(filepath));
    }
    void paint(int x, int y) {
        QPainter painter(&_image);
        painter.setPen(Qt::red);
        painter.drawPoint(x, y);
        painter.end();
        display(_image);
    }
    void paint(const gladys::point_xy_t& p, int step=1) {
        paint(p[0], p[1]);
    }
    void paint_point(const gladys::point_xy_t& p, int step=1) {
        paint(p, step);
    }
};

ImageViewer *image_viewer;
gladys::nav_graph *ng;
void paint(const gladys::point_xy_t& p, int step=1) {
    image_viewer->paint(p, step);
}

int main(int argc, char * argv[])
{
    if (argc < 3) {
        std::cerr<<"usage: gladys_gui region.tif robot.json"<<std::endl;
        return EXIT_FAILURE;
    }
    QApplication app(argc, argv);
    image_viewer = new ImageViewer;
    image_viewer->show();
    ng = new gladys::nav_graph(argv[1], argv[2]);
    image_viewer->display(ng->get_map());
    // TODO thread
    //std::thread t1(graph_thread);

    image_viewer->paint(50, 50);
    //gladys::display_hook_t func;
    //ng->set_display_hook( std::bind(&ImageViewer::paint_point, image_viewer, std::placeholders::_1, std::placeholders::_2) );

    gladys::point_xy_t p1 = {512, 1};
    gladys::point_xy_t p2 = {512, 512};//{(int)map.get_width(), (int)map.get_height()};
    gladys::path_t path = ng->astar_search(p1, p2);
    std::cout<<"path: "<<gladys::to_string(path)<<std::endl;
    for (const auto& p : path)
        image_viewer->paint(p);
    //t1.join();
    // run
    return app.exec();
}
