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

typedef std::vector<float> raster;

/*
 * gdal : GDALDataset wraper
 */
class gdal {
    typedef std::vector<raster> rasters;
    std::array<double, 6> transform;
    size_t x_size;
    size_t y_size;
    int utm_zone;
    bool utm_north;

    void _init() {
        // Register all known configured GDAL drivers.
        GDALAllRegister();
        set_transform(0, 0);
        set_utm(0);
    }

public:
    rasters bands;

    gdal() {
        _init();
    }

    gdal(const std::string& filepath) {
        _init();
        load(filepath);
    }

    /** Copy meta-data from another instance
     *
     * @param copy another gdal instance
     */
    void copy_meta(const gdal& copy) {
        utm_zone  = copy.utm_zone;
        utm_north = copy.utm_north;
        transform = copy.transform;
        set_size(copy.bands.size(), copy.x_size, copy.y_size);
    }

    /** Copy meta-data from another instance, except the number of layers
     *
     * @param copy another gdal instance
     * @param n_raster number of layers to set (number of rasters)
     */
    void copy_meta(const gdal& copy, size_t n_raster) {
        utm_zone  = copy.utm_zone;
        utm_north = copy.utm_north;
        transform = copy.transform;
        set_size(n_raster, copy.x_size, copy.y_size);
    }

    /** Set Universal Transverse Mercator projection definition.
     *
     * @param zone UTM zone.
     * @param north TRUE for northern hemisphere, or FALSE for southern.
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
        transform[0] = pos_x;   // top left x
        transform[1] = width;   // w-e pixel resolution
        transform[2] = 0.0;     // rotation, 0 if image is "north up"
        transform[3] = pos_y;   // top left y
        transform[4] = 0.0;     // rotation, 0 if image is "north up"
        transform[5] = height;  // n-s pixel resolution
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

    size_t get_x() const {
        return x_size;
    }

    size_t get_y() const {
        return y_size;
    }

    double get_scale_x() const {
        return transform[1]; // pixel width
    }

    double get_scale_y() const {
        return transform[5]; // pixel height
    }

    double get_utm_pose_x() const {
        return transform[0]; // upper left pixel position x
    }

    double get_utm_pose_y() const {
        return transform[3]; // upper left pixel position y
    }

    /** Save as GeoTiff
     *
     * @param filepath path to .tif file.
     */
    int save(const std::string& filepath) const {
        // get the GDAL GeoTIFF driver
        GDALDriver *driver = GetGDALDriverManager()->GetDriverByName("GTiff");
        if ( driver == NULL )
            throw std::runtime_error("[gdal] could not get the driver");

        // create the GDAL GeoTiff dataset (n layers of float32)
        GDALDataset *dataset = driver->Create( filepath.c_str(), x_size, y_size,
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

        GDALRasterBand *band;
        for (int band_id = 0; band_id < bands.size(); band_id++) {
            band = dataset->GetRasterBand(band_id+1);
            band->RasterIO( GF_Write, 0, 0, x_size, y_size,
                (void *) bands[band_id].data(), x_size, y_size, GDT_Float32, 0, 0 );
        }

        // close properly the dataset
        GDALClose( (GDALDatasetH) dataset );
        return EXIT_SUCCESS;
    }

    /** Load a GeoTiff
     *
     * @param filepath path to .tif file.
     */
    int load(const std::string& filepath) {
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

        GDALRasterBand *band;
        for (int band_id = 0; band_id < bands.size(); band_id++) {
            band = dataset->GetRasterBand(band_id+1);
            if ( band->GetRasterDataType() != GDT_Float32 )
                std::cerr<<"[warn] only support Float32 bands"<<std::endl;
            band->RasterIO( GF_Read, 0, 0, x_size, y_size,
                bands[band_id].data(), x_size, y_size, GDT_Float32, 0, 0 );
        }

        // close properly the dataset
        GDALClose( (GDALDatasetH) dataset );
        return EXIT_SUCCESS;
    }
};

// helpers

inline bool operator==( const gdal& lhs, const gdal& rhs ) {
    return (lhs.get_x() == rhs.get_x()
        and lhs.get_y() == rhs.get_y()
        and lhs.get_scale_x() == rhs.get_scale_x()
        and lhs.get_scale_y() == rhs.get_scale_y()
        and lhs.get_utm_pose_x() == rhs.get_utm_pose_x()
        and lhs.get_utm_pose_y() == rhs.get_utm_pose_y()
        and lhs.bands == rhs.bands );
}
inline std::string to_string(const gdal& value) {
    return "GDAL[" + std::to_string(value.get_x()) + "," +
                     std::to_string(value.get_y()) + "]";
}
inline std::ostream& operator<<(std::ostream& os, const gdal& value) {
    return os<<to_string(value);
}

} // namespace gladys

#endif // GDAL_HPP

