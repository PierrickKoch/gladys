/*
 * gdal.hpp
 *
 * Graph Library for Autonomous and Dynamic Systems
 *
 * author:  Pierrick Koch <pierrick.koch@laas.fr>
 * created: 2013-06-12
 * license: BSD
 */
#ifndef GDAL_HPP
#define GDAL_HPP

#include <string>
#include <vector>
#include <array>
#include <stdexcept>        // for runtime_error
#include <gdal_priv.h>      // for GDALDataset
#include <ogr_spatialref.h> // for OGRSpatialReference

namespace gladys {
    /*
     * gdal : GDALDataset wraper
     */
    class gdal {
        typedef std::vector<float> raster;
        typedef std::vector<raster> rasters;
        std::array<double, 6> transform;
        size_t x_size;
        size_t y_size;
        int utm_zone;
        bool utm_north;
    public:
        rasters bands;

        gdal() {
            // Register all known configured GDAL drivers.
            GDALAllRegister();
        }

        /** Set Universal Transverse Mercator projection definition.
         *
         * @param zone UTM zone.
         * @param north TRUE for northern hemisphere, or FALSE for southern hemisphere.
         */
        void set_utm(int zone, bool north = true) {
            utm_zone = zone;
            utm_north = north;
        }

        /** Set the coefficients for transforming
         * between pixel/line (P,L) raster space,
         * and projection coordinates (Xp,Yp) space.
         *
         * @param pos_x upper left pixel position x
         * @param pos_y upper left pixel position y
         * @param width pixel width (default 1.0)
         * @param height pixel height (default 1.0)
         */
        void set_transform(double pos_x, double pos_y,
                double width = 1.0, double height = 1.0) {
            transform[0] = pos_x;
            transform[1] = width;
            transform[2] = 0.0;
            transform[3] = pos_y;
            transform[4] = 0.0;
            transform[5] = height;
        }

        /** Set raster size.
         *
         * @param n number of rasters.
         * @param x number of columns.
         * @param y number of rows.
         */
        void set_size(size_t n, size_t x, size_t y) {
            x_size = x;
            y_size = y;
            bands.resize( n );
            size_t size = x * y;
            for (auto& band: bands)
                band.resize( size );
        }

        size_t get_x() {
            return x_size;
        }

        size_t get_y() {
            return y_size;
        }

        /** Save as GeoTiff
         *
         * @param filepath path to .tif file.
         */
        int save(const std::string& filepath) const {
            GDALDriver *driver;
            GDALDataset *dataset;
            GDALRasterBand *band;

            // get the GDAL GeoTIFF driver
            driver = GetGDALDriverManager()->GetDriverByName("GTiff");
            if ( driver == NULL )
                throw std::runtime_error("[gdal] could not get the driver");

            // create the GDAL GeoTiff dataset (n layers of float32)
            dataset = driver->Create( filepath.c_str(), x_size, y_size,
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

            for (int band_id = 0; band_id < bands.size(); band_id++) {
                band = dataset->GetRasterBand(band_id+1);
                band->RasterIO( GF_Write, 0, 0, x_size, y_size,
                    (void *) bands[band_id].data(), x_size, y_size, GDT_Float32, 0, 0 );
            }

            // close properly the dataset
            GDALClose( (GDALDatasetH) dataset );
            return 0;
        }

        /** Load a GeoTiff
         *
         * @param filepath path to .tif file.
         */
        int load(const std::string& filepath) {
            GDALDriver *driver;
            GDALDataset *dataset;
            GDALRasterBand *band;

            // Open a raster file as a GDALDataset.
            dataset = (GDALDataset *) GDALOpen( filepath.c_str(), GA_ReadOnly );
            if ( dataset == NULL )
                throw std::runtime_error("[gdal] could not open the given file");

            std::string _type = GDALGetDriverShortName( dataset->GetDriver() );
            if ( _type.compare( "GTiff" ) != 0 )
                std::cerr<<"[warn] expected GTiff and got: "<<_type<<std::endl;

            set_size( dataset->GetRasterCount(), dataset->GetRasterXSize(),
                dataset->GetRasterYSize() );
            // TODO get utm zone
            std::string _projection = dataset->GetProjectionRef();

            double _transform[6];
            if( dataset->GetGeoTransform( _transform ) != CE_None )
                std::cerr<<"[warn] no GeoTransform"<<std::endl;
            // TODO else
            //    transform.fill(_transform);

            for (int band_id = 0; band_id < bands.size(); band_id++) {
                band = dataset->GetRasterBand(band_id+1);
                if ( band->GetRasterDataType() != GDT_Float32 )
                    std::cerr<<"[warn] only support Float32 bands"<<std::endl;
                band->RasterIO( GF_Read, 0, 0, x_size, y_size,
                    bands[band_id].data(), x_size, y_size, GDT_Float32, 0, 0 );
            }

            // close properly the dataset
            GDALClose( (GDALDatasetH) dataset );
            return 0;
        }
    };
}

#endif // GDAL_HPP

