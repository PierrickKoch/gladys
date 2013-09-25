#!/usr/bin/env python

import json

try:
    from PySide import QtCore, QtGui
except ImportError:
    print("[error] sudo apt-get install python-pyside")
    import sys; sys.exit(1)

def avg(lst):
    tmp = 0.0
    for elt in lst:
        tmp += elt
    return float(tmp) / len(lst)

class ImageLabel(QtGui.QLabel):
    def __init__(self):
        QtGui.QLabel.__init__(self)
    def set_image(self, image):
        self.setPixmap(QtGui.QPixmap.fromImage(image))

class MainWindow(QtGui.QMainWindow):
    def __init__(self, argv):
        super(MainWindow, self).__init__()
        self.points_pix = []
        # image viewer
        self.image_label = ImageLabel()
        self.setCentralWidget(self.image_label)

        self.setWindowTitle("Eurasample")
        self.resize(200, 200)

        fmap = argv[1]
        futm = argv[2]
        fpnt = argv[3]
        self.path_image = "%s.out.png" % fmap

        self.image_map = QtGui.QImage(fmap)
        self.image_label.set_image(self.image_map)

        self.points_utm = []
        with open(futm) as fputm:
            buff = fputm.read().strip().splitlines()
            for line in buff:
                z,x,y,n = line.split()
                self.points_utm.append([float(x), float(y)])

        self.points_pnt = []
        with open(fpnt) as fputm:
            buff = fputm.read().strip().splitlines()[1:-1]
            for line in buff:
                d,y,p,r,x,y,z = line.split()
                self.points_pnt.append([float(x), float(y)])
        with open('/tmp/pom.json', 'w') as pomjson:
            pomjson.write(str(self.points_pnt))


        print("===============================\n"
              "  Welcome to Eurasample !\n"
              "===============================\n\n"
              "Actions\n"
              "-------\n"
              " - Click     = select points\n"
              " - C         = clear points\n"
              " - S         = save image\n"
              " - Space     = start utm\n"
              " - Escape    = quit\n")

        print("UTM points = %s" % str(self.points_utm) )

        # key bindings
        self._bindings = {}
        self.bind(QtCore.Qt.Key_Escape, self.close)
        self.bind(QtCore.Qt.Key_C,      self.clear_points)
        self.bind(QtCore.Qt.Key_Space,  self.start_utm)

    def bind(self, key, func):
        self._bindings[key] = func

    def clear_points(self):
        self.points_pix = []
        print("clear points")

    def start_utm(self):
        print("start UTM")
        print("points: %s" % str(self.points_pix))
        if len(self.points_pix) < len(self.points_utm):
            print("[ERROR] len(self.points_pix) < len(self.points_utm)")
            return
        self.compute_scale()
        self.compute_origin()
        self.draw_all_utm()
        self.save_image()
        self.clear_points()


    def compute_scale(self):
        scal_x = []
        scal_y = []
        for i in range(len(self.points_utm)-1):
            print("%s -> %s" % (self.points_pix[i], self.points_utm[i]) )
            dx_pix = self.points_pix[i][0] - self.points_pix[i+1][0]
            dy_pix = self.points_pix[i][1] - self.points_pix[i+1][1]
            dx_utm = self.points_utm[i][0] - self.points_utm[i+1][0]
            dy_utm = self.points_utm[i][1] - self.points_utm[i+1][1]
            # meters per pixel, aka. pixel size in meter
            scal_x.append( dx_utm / dx_pix )
            scal_y.append( dy_utm / dy_pix )
        self.scale_x = avg(scal_x)
        self.scale_y = avg(scal_y)
        print("scale: %s" % str((self.scale_x, self.scale_y)) )


    def compute_origin(self):
        orig_x = []
        orig_y = []
        for i in range(len(self.points_utm)):
            orx = self.points_utm[i][0] - self.points_pix[i][0] * self.scale_x
            ory = self.points_utm[i][1] - self.points_pix[i][1] * self.scale_y
            orig_x.append( orx )
            orig_y.append( ory )
            print(orx, ory)
        self.origin_x = avg(orig_x)
        self.origin_y = avg(orig_y)
        print("origin: %s" % str((self.origin_x, self.origin_y)) )

    def draw_all_utm(self):
        # if pom.log (else custom = 0)
        custom_origin_x = 354320.393534
        custom_origin_y = 5275994.529841
        # convert from custom to UTM
        points = [ [x + custom_origin_x, y + custom_origin_y] \
                  for x,y in self.points_pnt ]
        # convert from UTM to pixel
        points = [ [(x - self.origin_x) / self.scale_x, \
                    (y - self.origin_y) / self.scale_y] \
                  for x,y in points ]
        # draw points
        painter = QtGui.QPainter(self.image_map)
        painter.setPen(QtGui.QPen(QtCore.Qt.green, 1)) # <- here is the path size !
        painter.drawPoints(QtGui.QPolygonF([ QtCore.QPointF(x, y) for x, y in points ]))
        painter.end()
        # show image
        self.image_label.set_image(self.image_map)
        # dump points in UTM
        with open('%s.utm.txt'%self.path_image, 'w') as f:
            for x,y in points:
                f.write("%f %f\n"%(x,y))

    def save_image(self):
        self.image_map.save(self.path_image)
        print("image saved in %s" % self.path_image)

    # PySide.QtGui.QWidget.keyPressEvent(event)
    def keyPressEvent(self, event):
        if event.key() in self._bindings:
            self._bindings[event.key()]()

    # PySide.QtGui.QWidget.mousePressEvent(event)
    def mousePressEvent(self, event):
        pose = event.pos()
        point = [pose.x(), pose.y()]
        if self.points_pix and self.points_pix[-1] == point:
            return # skip double click
        self.points_pix.append(point)
        print("new point: %s" % str(point) )

def main(argv=[]):
    if len(argv) < 4:
        print("usage: %s sample.png sample.txt utm.txt"%argv[0])
        return 1
    app = QtGui.QApplication(argv)
    mww = MainWindow(argv)
    mww.show()
    return app.exec_()

if __name__ == '__main__':
    import sys
    sys.exit( main(sys.argv) )

