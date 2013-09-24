/*
 * gdal.cpp
 *
 * Graph Library for Autonomous and Dynamic Systems
 *
 * author:  Pierrick Koch <pierrick.koch@laas.fr>
 * created: 2013-09-22
 * license: BSD
 */

#include <string>
#include <iostream>         // cout,cerr,endl
#include <stdexcept>        // for runtime_error
#include <gdal_priv.h>      // for GDALDataset
#include <ogr_spatialref.h> // for OGRSpatialReference

#include "gladys/gdal.hpp"

namespace gladys {

void gdal::_init() {
    // Register all known configured GDAL drivers.
    GDALAllRegister();
    set_transform(0, 0);
    set_custom_origin(0, 0);
    set_utm(0);
}

/** Save as GeoTiff
 *
 * @param filepath path to .tif file.
 */
void gdal::save(const std::string& filepath) const {
    // get the GDAL GeoTIFF driver
    GDALDriver *driver = GetGDALDriverManager()->GetDriverByName("GTiff");
    if ( driver == NULL )
        throw std::runtime_error("[gdal] could not get the driver");

    // create the GDAL GeoTiff dataset (n layers of float32)
    GDALDataset *dataset = driver->Create( filepath.c_str(), width, height,
        bands.size(), GDT_Float32, NULL );
    if ( dataset == NULL )
        throw std::runtime_error("[gdal] could not create (multi-layers float32)");

    // set the projection
    OGRSpatialReference spatial_reference;
    char *projection = NULL;

    spatial_reference.SetUTM( utm_zone, utm_north );
    spatial_reference.SetWellKnownGeogCS( "WGS84" );
    spatial_reference.exportToWkt( &projection );
    dataset->SetProjection( projection );
    CPLFree( projection );

    // see GDALDataset::GetGeoTransform()
    dataset->SetGeoTransform( (double *) transform.data() );
    dataset->SetMetadataItem("CUSTOM_X_ORIGIN", std::to_string(custom_x_origin).c_str());
    dataset->SetMetadataItem("CUSTOM_Y_ORIGIN", std::to_string(custom_y_origin).c_str());

    GDALRasterBand *band;
    for (int band_id = 0; band_id < bands.size(); band_id++) {
        band = dataset->GetRasterBand(band_id+1);
        band->RasterIO( GF_Write, 0, 0, width, height,
            (void *) bands[band_id].data(), width, height, GDT_Float32, 0, 0 );
        band->SetMetadataItem("NAME", names[band_id].c_str());
    }

    // close properly the dataset
    GDALClose( (GDALDatasetH) dataset );
}

/** Load a GeoTiff
 *
 * @param filepath path to .tif file.
 */
void gdal::load(const std::string& filepath) {
    // Open a raster file as a GDALDataset.
    GDALDataset *dataset = (GDALDataset *) GDALOpen( filepath.c_str(), GA_ReadOnly );
    if ( dataset == NULL )
        throw std::runtime_error("[gdal] could not open the given file");

    std::string _type = GDALGetDriverShortName( dataset->GetDriver() );
    if ( _type.compare( "GTiff" ) != 0 )
        std::cerr<<"[warn] expected GTiff and got: "<<_type<<std::endl;

    set_size( dataset->GetRasterCount(), dataset->GetRasterXSize(),
        dataset->GetRasterYSize() );

    // get utm zone
    OGRSpatialReference spatial_reference( dataset->GetProjectionRef() );
    int _north;
    utm_zone = spatial_reference.GetUTMZone( &_north );
    utm_north = (_north != 0);

    // GetGeoTransform returns CE_Failure if the transform is not found
    // as well as when it's the default {0.0, 1.0, 0.0, 0.0, 0.0, 1.0}
    // and write {0.0, 1.0, 0.0, 0.0, 0.0, 1.0} in transform anyway
    // so error handling here is kind of useless...
    dataset->GetGeoTransform( transform.data() );
    const char *cxo = dataset->GetMetadataItem("CUSTOM_X_ORIGIN");
    const char *cyo = dataset->GetMetadataItem("CUSTOM_Y_ORIGIN");
    if (cxo != NULL)
        custom_x_origin = std::atof(cxo);
    if (cyo != NULL)
        custom_y_origin = std::atof(cyo);

    GDALRasterBand *band;
    const char *name;
    for (int band_id = 0; band_id < bands.size(); band_id++) {
        band = dataset->GetRasterBand(band_id+1);
        if ( band->GetRasterDataType() != GDT_Float32 )
            std::cerr<<"[warn] only support Float32 bands"<<std::endl;
        band->RasterIO( GF_Read, 0, 0, width, height,
            bands[band_id].data(), width, height, GDT_Float32, 0, 0 );
        name = band->GetMetadataItem("NAME");
        if (name != NULL)
            names[band_id] = name;
    }

    // close properly the dataset
    GDALClose( (GDALDatasetH) dataset );
}

} // namespace gladys
