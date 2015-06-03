/*
 * region_from_rgba.cpp
 *
 * Common LAAS Raster library
 *
 * author:  Pierrick Koch <pierrick.koch@laas.fr>
 * created: 2013-06-12
 * license: BSD
 */
#include <string>           // for string
#include <cstdlib>          // exit status

#include "gdalwrap/gdal.hpp"

/**
 * Standalone compile, just install `gdalwrap`
 * g++ $0.cpp -o $0 -std=c++0x `pkg-config --libs --cflags gdalwrap`
 *
 * Pipeline from dsm->edit->rgb->region
 * # create a dsm.png.aux.xml containing the geodata
 * gdal_translate -of PNG dsm.tif dsm.png
 * gimp dsm.png # color Red = Obstacle, Green = FLAT, Blue = ROUGH.
 * # recreate GeoTiff from the gimp file and the .aux.xml
 * gdal_translate -of GTiff dsm.png dsm.rgba.tif
 * # create a valid region-map GeoTiff
 * region_from_rgba dsm.rgba.tif dsm.rgba.region.tif
 */

void color_to_proba(const gdalwrap::raster& band_from,
                          gdalwrap::raster& band_to  ) {
    for (size_t idx = 0; idx < band_to.size(); idx++)
        band_to[idx] = band_from[idx] / 255.0;
}

int main(int argc, char * argv[])
{
    if (argc < 3) {
        std::cerr<<"usage: "<<argv[0]<<" rgba.tif region.tif"<<std::endl;
        return EXIT_FAILURE;
    }
    gdalwrap::gdal rgba  (argv[1]);
    gdalwrap::gdal region;
    region.names = {"NO_3D_CLASS", "FLAT", "OBSTACLE", "ROUGH"};
    region.copy_meta(rgba, region.names.size());
    color_to_proba(rgba.bands[0], region.get_band("OBSTACLE")); // red
    color_to_proba(rgba.bands[1], region.get_band("FLAT"));     // green
    color_to_proba(rgba.bands[2], region.get_band("ROUGH"));    // blue

    region.save(argv[2]);

    return EXIT_SUCCESS;
}
