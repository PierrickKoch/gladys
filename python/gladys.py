from libgladys_python import *

def point_custom2utm(gdal, x, y):
    return (x + gdal.get_custom_x_origin(),
            y + gdal.get_custom_y_origin())

def point_utm2custom(gdal, x, y):
    return (x - gdal.get_custom_x_origin(),
            y - gdal.get_custom_y_origin())

def point_pix2utm(gdal, x, y):
    return ( x * gdal.get_scale_x() + gdal.get_utm_pose_x() ,
             y * gdal.get_scale_y() + gdal.get_utm_pose_y() )

def point_utm2pix(gdal, x, y):
    return ((x - gdal.get_utm_pose_x()) / gdal.get_scale_x(),
            (y - gdal.get_utm_pose_y()) / gdal.get_scale_y())

def point_pix2custom(gdal, x, y):
    return point_utm2custom(gdal, *point_pix2utm(gdal, x, y))

def point_custom2pix(gdal, x, y):
    return point_utm2pix(gdal, *point_custom2utm(gdal, x, y))

class Gladys:
    def __init__(self, region, robot):
        self.ctmap = costmap(region, robot)
        self.graph = nav_graph(self.ctmap)
        self.gdmap = self.ctmap.get_map()
    def u2p(self, x, y):
        return point_utm2pix(self.gdmap, x, y)
    def p2u(self, x, y):
        return point_pix2utm(self.gdmap, x, y)
    def c2p(self, x, y):
        return point_custom2pix(self.gdmap, x, y)
    def p2c(self, x, y):
        return point_pix2custom(self.gdmap, x, y)
    def c2u(self, x, y):
        return point_custom2utm(self.gdmap, x, y)
    def u2c(self, x, y):
        return point_utm2custom(self.gdmap, x, y)
    def path(self, a, b):
        # A* a, b
        res_utm, cost = self.graph.search_with_cost(self.c2u(*a), self.c2u(*b))
        res = [self.u2c(x, y) for x, y in res_utm]
        return res, cost
