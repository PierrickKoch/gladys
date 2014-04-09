#! /usr/bin/env python

import json
import gdal

def point_utm2pix(x, y):
    return [(x - utm_x) / scale_x,
            (y - utm_y) / scale_y]

def point_custom2pix(x, y):
    return point_utm2pix(
            x + custom_x_origin,
            y + custom_y_origin )

def get_points(geofile, radius):
    # get GeoTiff for scale info
    geotiff = gdal.Open(geofile)
    tf      = geotiff.GetGeoTransform()
    meta    = geotiff.GetMetadata()
    band    = geotiff.GetRasterBand(1)
    image   = band.ReadAsArray()
    width   = geotiff.RasterXSize
    height  = geotiff.RasterYSize
    scale_x = tf[1]
    scale_y = tf[5]
    utm_x   = tf[0]
    utm_y   = tf[3]
    custom_x_origin = float(meta['CUSTOM_X_ORIGIN'])
    custom_y_origin = float(meta['CUSTOM_Y_ORIGIN'])

    def point_pix2utm(x, y):
        return [ x * scale_x + utm_x ,
                 y * scale_y + utm_y ]

    def point_pix2custom(x, y):
        p = point_pix2utm(x, y)
        return [p[0] - custom_x_origin,
                p[1] - custom_y_origin]

    # get robot radius (in meter)
    radius  = float(radius)
    xradius = int(radius/abs(scale_x))
    yradius = int(radius/abs(scale_y))

    points = []
    for i in range(xradius, width, xradius):
        for j in range(yradius, height, yradius):
            if image[j,i] == 255:
                points.append((i, j))

    return [point_pix2custom(*p) for p in points]

## in ipython
# implot = plt.imshow(image)
# x,y = zip(*points)
# plt.plot(x, y, 'o', c='g')
## in gmaps
# function padd(p) { add(p[0], p[1]); }
# points.forEach(padd)

def main(argv=[]):
    if len(argv) < 3:
        print("usage: %s geo.png radius"%argv[0])
        return 1

    print(json.dumps( get_points(argv[1], argv[2]) ))
    return 0

if __name__ == '__main__':
    import sys
    sys.exit( main(sys.argv) )
