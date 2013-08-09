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

#include <QtGui>

#include "gladys/display.hpp"

QtAppStart::QtAppStart(int argc, char * argv[], void (*_display)(ImageViewer *)) : display_(_display) {
    QApplication app(argc, argv);
    image_viewer = new ImageViewer;
    image_viewer->show();
    //QTimer::singleShot(0, this, SLOT(display()));
    display();
    app.exec();
}

void QtAppStart::display() {
    (*display_)(image_viewer);
}
