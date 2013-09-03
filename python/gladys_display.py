#!/usr/bin/env python

import time
import struct
import colorsys

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

class ImageLabel(QtGui.QLabel):
    pixmap = QtCore.Signal(QtGui.QImage)
    def __init__(self):
        QtGui.QLabel.__init__(self)
        self.path = None
        self.pixmap.connect(self._set_image)
    def _set_image(self, image):
        self.setPixmap(QtGui.QPixmap.fromImage(image))
    def paintEvent(self, event):
        QtGui.QLabel.paintEvent(self, event)
        if self.path:
            painter = QtGui.QPainter(self)
            painter.setPen(QtGui.QPen(QtCore.Qt.black, 5))
            painter.drawPoints(self.path)

def get_bgra32(b=0,g=0,r=0,a=0):
    return struct.pack('4B', int(b*255), int(g*255), int(r*255), int(a*255))

def float_to_bgra32(hue):
    hue = (1 - hue) / 1.5
    r,g,b = colorsys.hsv_to_rgb(hue, 1, 1)
    return get_bgra32(b,g,r,1)

class MainWindow(QtGui.QMainWindow):
    def __init__(self, argv):
        super(MainWindow, self).__init__()
        self.ng = self.start = self.end = None
        # image viewer
        self.image_label = ImageLabel()
        self.setCentralWidget(self.image_label)
        self.setWindowTitle("gladys display")
        self.resize(200, 200)
        self.show()
        self.fregion = argv[1]
        self.frobot = argv[2]
        self._bindings = {}
        self.bind(QtCore.Qt.Key_Escape, self.close)
        self.bind(QtCore.Qt.Key_Space,  self.start_search)
        self.bind(QtCore.Qt.Key_C,      self.clear_points)
        # time load_nav_graph
        QtCore.QTimer.singleShot(50, self.load_nav_graph)
    def show_region(self, width, height, fdata):
        buff = []
        fmin = min(fdata)
        diff = max(fdata) - fmin
        print("min: %f, diff: %f" % (fmin, diff))
        if not diff: # useless
            print("max == min (monochrome) : useless")
            return
        for fpx in fdata:
            #try:
            #    int(fpx)
            #except: # if is NaN or inf
            #    fpx = 255
            hue = (fpx - fmin) / diff
            buff.append(float_to_bgra32(hue))
        imageARGB32 = b''.join(buff)
        self.update_image(width, height, imageARGB32)
    def load_nav_graph(self):
        self.ng = gladys.nav_graph(self.fregion, self.frobot)
        re = self.ng.get_map().get_region()
        self.show_region(re.get_width(), re.get_height(), re.get_band('FLAT'))
    def update_image(self, width, height, imageARGB32):
        image = QtGui.QImage(imageARGB32, width, height, QtGui.QImage.Format_ARGB32)
        self.image_label.pixmap.emit( image.copy() )
    def update_path(self, path):
        qp = QtGui.QPolygonF(len(path))
        for x, y in path:
            qp.append(QtCore.QPointF(x, y))
        self.image_label.path = qp

    def search(self):
        if not self.ng:
            print("not ready for search (no graph)")
        print("search: %s" % str((self.start, self.end)) )
        pp = self.ng.search(self.start, self.end)
        print("path: %s" % str(pp) )
        self.update_path(pp)
        self.start = self.end = None

    def bind(self, key, func):
        self._bindings[key] = func

    def start_search(self):
        if self.start and self.end:
            QtCore.QTimer.singleShot(50, self.search)
        else:
            print("select start and end points")
    def clear_points(self):
        self.start = self.end = None
        print("clear start and end")
    def close(self):
        QtGui.QMainWindow.close(self)

    # PySide.QtGui.QWidget.keyPressEvent(event)
    def keyPressEvent(self, event):
        if event.key() in self._bindings:
            self._bindings[event.key()]()
    # PySide.QtGui.QWidget.mousePressEvent(event)
    def mousePressEvent(self, event):
        if not self.start:
            self.start = ( event.x(), event.y() )
            print("start = %s" % str(self.start) )
        if not self.end:
            self.end = ( event.x(), event.y() )
            if self.end == self.start:
                self.end = None
            else:
                print("end = %s" % str(self.end) )
        self.image_label.repaint()

def main(argv=[]):
    if len(argv) < 3:
        print("usage: %s region.tif robot.json"%argv[0])
        return 1
    app = QtGui.QApplication(argv)
    mw = MainWindow(argv)
    return app.exec_()

if __name__ == '__main__':
    import sys
    sys.exit( main(sys.argv) )

