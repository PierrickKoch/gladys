#!/usr/bin/env python
"""
git clone git://github.com/OSGeo/gdal.git
cd gdal/gdal/swig/python/
git checkout tags/1.10.0
python setup.py install

git clone git://github.com/numpy/numpy.git
cd numpy/
git checkout v1.7.1
python setup.py install

git clone git://github.com/scipy/scipy.git
cd scipy/
git checkout v0.12.0
python setup.py install
"""
import gdal
import numpy
import scipy.misc
import colorsys # python/Lib
import json # python/Lib

def float_to_rgb(hue):
    assert(0.0 <= hue <= 1.0)
    # from blue to red (instead of red to red)
    # aka from 240 to 0 instead of 0 to 360 degrees (HSV)
    hue = (1 - hue) / 1.5
    return colorsys.hsv_to_rgb(hue, 1, 1)

def save_numpy(filepath, numpy_array):
    scipy.misc.imsave(filepath, numpy_array)

def gdal_float_to_rgb(file_in, file_out, band_id=1):
    img = gdal.Open(file_in)
    if band_id < 0:
        for band_id in range(1, img.RasterCount + 1):
            gdal_img_float_to_rgb(img, '%s.%i.png'%(file_out, band_id), band_id)
    else:
        gdal_img_float_to_rgb(img, file_out, band_id)

def gdal_img_float_to_rgb(img, file_out, band_id):
    band = img.GetRasterBand(band_id)
    width = band.XSize
    height = band.YSize
    raster = band.ReadAsArray() # numpy.ndarray
    # in some case gdal returns None for min/max, so use numpy
    mini = band.GetMinimum() or raster.min()
    maxi = band.GetMaximum() or raster.max()
    print("min: %f max: %f file: %s" % (mini, maxi, file_out) )
    diff = maxi - mini
    if diff == 0: return # div zero = NaN (useless band)
    raster_rgb = numpy.zeros((height, width, 3), 'uint8')
    print("float to rgb for each pixel (slow)...")
    for x in range(width):
        for y in range(height):
            hue = (raster[y][x] - mini) / diff
            if hue > 1: hue = 1
            r,g,b = float_to_rgb(hue)
            raster_rgb[y][x][0] = r * 255
            raster_rgb[y][x][1] = g * 255
            raster_rgb[y][x][2] = b * 255
    print("done! saving...")
    save_numpy(file_out, raster_rgb)
    with open('%s.json'%file_out, 'w') as f:
        json.dump({'min':mini,'max':maxi},f)

def main(argv=[]):
    if len(argv) < 3:
        print("usage: %s file.tif file.png [band]" % argv[0])
        return 1
    band = 1
    if len(argv) > 3:
        band = int(argv[3])
    gdal_float_to_rgb(argv[1], argv[2], band)
    return 0

if __name__ == '__main__':
    import sys
    sys.exit(main(sys.argv))
