try: # relative import
    from .libgladys_python import *
except ValueError:
    from libgladys_python import *


def point_pix2utm(gdal, x, y):
    return ( x * gdal.get_scale_x() + gdal.get_utm_pose_x() ,
             y * gdal.get_scale_y() + gdal.get_utm_pose_y() )

def point_utm2pix(gdal, x, y):
    return ((x - gdal.get_utm_pose_x()) / gdal.get_scale_x(),
            (y - gdal.get_utm_pose_y()) / gdal.get_scale_y())
