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

def float_to_rgba(hue):
    hue = (1 - hue) / 1.5
    r,g,b = colorsys.hsv_to_rgb(hue, 1, 1)
    return (int(r*255), int(g*255), int(b*255), 255)

class MainWindow(QtGui.QMainWindow):
    def __init__(self, argv):
        super(MainWindow, self).__init__()
        # image viewer
        self.image_label = ImageLabel()
        self.setCentralWidget(self.image_label)
        self.setWindowTitle("gladys display")
        self.resize(200, 200)
        self.show()
        self.gladys = Gladys(self, argv[1], argv[2])
        self.gladys.start()
    def updateImage(self, width, height, imageARGB32):
        image = QtGui.QImage(imageARGB32, width, height, QtGui.QImage.Format_ARGB32)
        self.image_label.pixmap.emit( image.copy() )
    def updatePath(self, path):
        qp = QtGui.QPolygonF(len(path))
        for x, y in path:
            qp.append(QtCore.QPointF(x, y))
        self.image_label.path = qp
        #self.image_label.repaint()
    def close(self):
        QtGui.QMainWindow.close(self)
    def mousePressEvent(self, event):
        self.image_label.repaint()

class Gladys(QtCore.QThread):
    def __init__(self, mainwin, fregion, frobot):
        QtCore.QThread.__init__(self, mainwin)
        self.fregion = fregion
        self.frobot = frobot
        self.mainwin = mainwin
    def showPath(self, path):
        self.mainwin.updatePath(path)
    def showRegion(self, width, height, fdata):
        cdata = []
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
            cdata.extend(float_to_rgba(hue))
        imageARGB32 = struct.pack('%iB'%len(cdata), *cdata)
        self.mainwin.updateImage(width, height, imageARGB32)
    def run(self):
        ng = gladys.nav_graph(self.fregion, self.frobot)
        wm = ng.get_map()
        re = wm.get_region()
        self.showRegion(re.get_width(), re.get_height(), re.get_band('FLAT'))
        pp = ng.search( (0, 0), (re.get_width(), re.get_height()) )
        print("path: %s" % str(pp) )
        self.showPath(pp)

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

