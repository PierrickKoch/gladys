#!/usr/bin/env python

import json

try:
    from PySide import QtCore, QtGui
except ImportError:
    print("[error] sudo apt-get install python-pyside")
    import sys; sys.exit(1)

try:
    import gladys
except ImportError:
    print("[error] install gladys [and setup PYTHONPATH]")
    import sys; sys.exit(1)

def draw_path(paintable, path, sx=1.0, sy=1.0):
    painter = QtGui.QPainter(paintable)
    painter.scale(sx, sy)
    painter.setPen(QtGui.QPen(QtCore.Qt.green, 4))
    # draw path
    painter.drawPolyline(path)
    painter.setPen(QtGui.QPen(QtCore.Qt.red, 4))
    for num, point in enumerate(path):
        painter.drawText(point, "%i"%num)
    painter.end()

def draw_points(paintable, points, sx=1.0, sy=1.0):
    painter = QtGui.QPainter(paintable)
    painter.scale(sx, sy)
    painter.setPen(QtGui.QPen(QtCore.Qt.red, 4))
    # draw points
    painter.drawPoints(points)
    painter.end()

class ImageLabel(QtGui.QLabel):
    def __init__(self):
        QtGui.QLabel.__init__(self)
        self.path   = QtGui.QPolygonF()
        self.points = QtGui.QPolygonF()
        self.scale  = 1.0
    def set_image(self, image):
        self.image = image
        size = image.size() * self.scale
        self.setPixmap(QtGui.QPixmap.fromImage( image.scaled(size) ))
    def add_point(self, x, y):
        self.points.append(QtCore.QPointF(x, y))
        self.repaint()
    def clear_points(self):
        self.points = QtGui.QPolygonF()
        self.repaint()
    def paint_path(self, path):
        self.path = QtGui.QPolygonF([ QtCore.QPointF(x, y) for x, y in path ])
        self.repaint()
    def apply_scale(self, scale=None):
        if scale:
            self.scale = scale
        self.set_image(self.image)
        self.repaint()
    def paintEvent(self, event):
        # override QtGui.QLabel.paintEvent
        # called by QtGui.QLabel.repaint, in the GUI loop
        QtGui.QLabel.paintEvent(self, event)
        draw_path  (self, self.path,   self.scale, self.scale)
        draw_points(self, self.points, self.scale, self.scale)

class MainWindow(QtGui.QMainWindow):
    def __init__(self, argv):
        super(MainWindow, self).__init__()
        # image viewer
        self.image_label = ImageLabel()
        self.setCentralWidget(self.image_label)

        self.setWindowTitle("Eurasample")
        self.resize(200, 200)

        self.points_pix = []
        self.image_gdal = gladys.gdal(argv[1])
        self.image_disp = QtGui.QImage(argv[1])
        self.image_label.set_image(self.image_disp)

        print("===============================\n"
              "  Welcome to Display UTM !\n"
              "===============================\n\n"
              "Actions\n"
              "-------\n"
              " - Click     = select points\n"
              " - C         = clear points\n"
              " - Space     = get points UTM\n"
              " - Escape    = quit\n")

        # key bindings
        self._bindings = {}
        self.bind(QtCore.Qt.Key_Escape, self.close)
        self.bind(QtCore.Qt.Key_C,      self.clear_points)
        self.bind(QtCore.Qt.Key_Space,  self.get_utm_coord)
        self.bind(QtCore.Qt.Key_Q,      self.zoom_in)
        self.bind(QtCore.Qt.Key_A,      self.zoom_out)

    def zoom_in(self):
        self.image_label.apply_scale(2.0)

    def zoom_out(self):
        self.image_label.apply_scale(0.5)

    def point_pix2utm(self, point):
        return gladys.point_pix2utm(self.image_gdal, *point)

    def point_pix2custom(self, point):
        return gladys.point_pix2custom(self.image_gdal, *point)

    def get_utm_coord(self):
        utm_coordinates = []
        custom_coordinates = []
        for point in self.points_pix:
            utm    = self.point_pix2utm(point)
            custom = self.point_pix2custom(point)
            utm_coordinates.append(utm)
            custom_coordinates.append(custom)
            print("[%5i, %5i] -> [%12.3f, %12.3f] -> [%9.3f, %9.3f]" % \
                  ( point[0], point[1], utm[0], utm[1], custom[0], custom[1] ) )
        # display path
        self.image_label.paint_path(self.points_pix)
        self.save_custom(custom_coordinates)
        self.save_image()

    def save_image(self):
        image = self.image_disp.copy()
        path  = self.image_label.path
        draw_path(image, path)
        image.save('/tmp/path.png')

    def save_custom(self, coord):
        with open('/tmp/path.txt', 'w') as f:
            for x, y in coord:
                f.write("goto %f %f\n" % (x, y) )

    def clear_points(self):
        self.points_pix = []
        self.image_label.paint_path([])
        self.image_label.clear_points()

    def bind(self, key, func):
        self._bindings[key] = func

    # PySide.QtGui.QWidget.keyPressEvent(event)
    def keyPressEvent(self, event):
        if event.key() in self._bindings:
            self._bindings[event.key()]()

    # PySide.QtGui.QWidget.wheelEvent(event)
    def wheelEvent(self, event):
        # TODO
        self.image_label.scale += event.delta() / 400.0;
        self.image_label.apply_scale()

    # PySide.QtGui.QWidget.mousePressEvent(event)
    def mousePressEvent(self, event):
        pose = event.pos()
        point = [pose.x(), pose.y()]
        if self.points_pix and self.points_pix[-1] == point:
            return # skip double click
        self.points_pix.append(point)
        self.image_label.add_point(*point)

def main(argv=[]):
    if len(argv) < 2:
        print("usage: %s geoimage"%argv[0])
        return 1
    app = QtGui.QApplication(argv)
    mww = MainWindow(argv)
    mww.show()
    return app.exec_()

if __name__ == '__main__':
    import sys
    sys.exit( main(sys.argv) )

