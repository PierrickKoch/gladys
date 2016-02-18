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
