/*
 * display.hpp
 *
 * Tool for Graph Library for Autonomous and Dynamic Systems
 *
 * author:  Pierrick Koch <pierrick.koch@laas.fr>
 * created: 2013-07-31
 * license: BSD
 */
#include <string>
#include <deque>

#include <QtGui>

#include "gladys/gdal.hpp"
#include "gladys/point.hpp"

typedef unsigned char uchar;
typedef std::vector<uchar> image8u_t;

// https://qt-project.org/doc/qt-4.8/widgets-imageviewer.html
// http://qt-project.org/doc/qt-4.8/qcolor.html#setHsv

class ImageViewer : public QMainWindow {
Q_OBJECT
private:
    QLabel *_image_label;
    QImage _image;
    bool update_needed;
    bool updating;
public:
    ImageViewer() {
        setWindowTitle(tr("Image Viewer"));
        _image_label = new QLabel;
        setCentralWidget(_image_label);
        update_needed = false;
        updating = false;
    }
    void update_image() {
        // must be in GUI thread
        if (update_needed) {
            updating = true;
            display(QPixmap::fromImage(_image));
            update_needed = false;
            updating = false;
        }
    }
    void display(const QPixmap& pixmap) {
        // must be in GUI thread
        _image_label->setPixmap(pixmap);
        // refresh
        _image_label->repaint();
        //_image_label->update();
        _image_label->adjustSize();
        this->adjustSize();
    }
    void display(const QImage& image) {
        if (!updating) {
            _image = image;
            update_needed = true;
        }
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
    void paint(int x, int y, int step=1) {
        QPainter painter(&_image);
        if (step == 1)
            painter.setPen(Qt::red);
        else if (step == 2)
            painter.setPen(Qt::green);
        else
            painter.setPen(Qt::blue);
        painter.drawPoint(x, y);
        painter.end();
        display(_image);
    }
    void paint(const gladys::point_xy_t& p, int step=1) {
        paint(p[0], p[1], step);
    }
    void paint_point(const gladys::point_xy_t& p, int step=1) {
        paint(p, step);
    }
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
};

class QtAppStart: public QObject {
Q_OBJECT
private:
    QApplication *app;
    ImageViewer *image_viewer;
    void (*display_)(ImageViewer *);
public:
    QtAppStart(int argc, char * argv[], void (*_display)(ImageViewer *)) : display_(_display) {
        app = new QApplication(argc, argv);
        image_viewer = new ImageViewer;
        image_viewer->show();
        QTimer::singleShot(0, this, SLOT(update_image()));
        QtConcurrent::run(display_, image_viewer);
    }
    int exec() {
        return app->exec();
    }
    ~QtAppStart() {
        delete image_viewer;
        delete app;
    }
public slots:
    void update_image() {
        while (true) {
            image_viewer->update_image();
            //QThread::sleep(10);
        }
    }
};
