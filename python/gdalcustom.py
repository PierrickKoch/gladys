#!/usr/bin/env python

try:
    import gladys
except ImportError:
    print("[error] install gladys [and setup PYTHONPATH]")
    import sys; sys.exit(1)

def main(argv=[]):
    if len(argv) < 4:
        print("usage: %s geo.tif UTM_X UTM_Y"%argv[0])
        return 1

    # get GeoTiff for scale info
    geotiff = gladys.gdal(argv[1])
    # get UTM X
    utm_x   = float(argv[2])
    # get UTM Y
    utm_y   = float(argv[3])

    geotiff.set_custom_origin(utm_x, utm_y)
    geotiff.save(argv[1])

    return 0

if __name__ == '__main__':
    import sys
    sys.exit( main(sys.argv) )
