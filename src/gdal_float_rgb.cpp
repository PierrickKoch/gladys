/*
 * gdal_float_rgb.cpp
 *
 * Tool for Graph Library for Autonomous and Dynamic Systems
 *
 * author:  Pierrick Koch <pierrick.koch@laas.fr>
 * created: 2013-07-29
 * license: BSD
 */
#include <ostream> // standard C error stream
#include <cstdlib> // exit status
#include <cstdio>  // fopen
#include <algorithm> // minmax
#include <string>
#include <vector>
#include <map>

#include <png.h>

#include "gladys/gdal.hpp"

typedef struct {
    float r,g,b;
} rgb_t;
typedef struct {
    float h,s,v;
} hsv_t;

// from http://hg.python.org/cpython/file/3.3/Lib/colorsys.py
rgb_t hsv_to_rgb(hsv_t in) {
    int i;
    float f, p, q, t;
    rgb_t out;
    if (in.s == 0) {
        out.r = out.g = out.b = in.v;
        return out;
    }
    i = (int)(in.h*6.0);
    f = (in.h*6.0) - i;
    p = in.v*(1.0 - in.s);
    q = in.v*(1.0 - in.s*f);
    t = in.v*(1.0 - in.s*(1.0-f));
    i = i%6;
    switch(i) {
    case 0:
        out.r = in.v;
        out.g = t;
        out.b = p;
        break;
    case 1:
        out.r = q;
        out.g = in.v;
        out.b = p;
        break;
    case 2:
        out.r = p;
        out.g = in.v;
        out.b = t;
        break;

    case 3:
        out.r = p;
        out.g = q;
        out.b = in.v;
        break;
    case 4:
        out.r = t;
        out.g = p;
        out.b = in.v;
        break;
    case 5:
    default:
        out.r = in.v;
        out.g = p;
        out.b = q;
        break;
    }
    return out;
}

rgb_t float_to_rgb(float hue) {
    // assert(0.0 <= hue <= 1.0)
    // from blue to red (instead of red to red)
    // aka from 240 to 0 instead of 0 to 360 degrees (HSV)
    hsv_t in;
    in.s = in.v = 1;
    in.h = (1 - hue) / 1.5;
    return hsv_to_rgb(in);
}

typedef std::vector<std::vector<png_byte>> png_rows_t;
typedef std::map<std::string, std::string> png_text_t;
int save_img(const std::string& filename, const png_rows_t& img, size_t width, size_t height, const png_text_t& text)
{
    int code = 0;
    FILE *fp;
    png_structp png_ptr;
    png_infop info_ptr;
    
    // Open file for writing (binary mode)
    fp = std::fopen(filename.c_str(), "wb");
    if (fp == NULL) {
        fprintf(stderr, "Could not open file %s for writing\n", filename.c_str());
        code = 1;
        goto finalise;
    }

    // Initialize write structure
    png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
    if (png_ptr == NULL) {
        fprintf(stderr, "Could not allocate write struct\n");
        code = 1;
        goto finalise;
    }

    // Initialize info structure
    info_ptr = png_create_info_struct(png_ptr);
    if (info_ptr == NULL) {
        fprintf(stderr, "Could not allocate info struct\n");
        code = 1;
        goto finalise;
    }

    // Setup Exception handling
    if (setjmp(png_jmpbuf(png_ptr))) {
        fprintf(stderr, "Error during png creation\n");
        code = 1;
        goto finalise;
    }

    png_init_io(png_ptr, fp);

    // Write header (8 bit colour depth)
    png_set_IHDR(png_ptr, info_ptr, width, height,
            8, PNG_COLOR_TYPE_RGB, PNG_INTERLACE_NONE,
            PNG_COMPRESSION_TYPE_BASE, PNG_FILTER_TYPE_BASE);

    // Set metadata (Title, Author, Desciption, ... and customs)
    // see http://www.libpng.org/pub/png/spec/1.2/PNG-Chunks.html#C.Anc-text
    if ( ! text.empty() ) {
        std::vector<png_text> vtext(text.size());
        int id_text = 0;
        for (const auto& kv : text) {
            vtext[id_text].compression = PNG_TEXT_COMPRESSION_NONE;
            vtext[id_text].key = (png_charp) kv.first.data();
            vtext[id_text].text = (png_charp) kv.second.data();
            id_text++;
        }
        png_set_text(png_ptr, info_ptr, vtext.data(), vtext.size());
    }

    png_write_info(png_ptr, info_ptr);

    // Write image data
    for (const auto& row : img)
        png_write_row(png_ptr, (png_bytep)row.data());

    // End write
    png_write_end(png_ptr, NULL);

    finalise:
    if (fp != NULL) fclose(fp);
    if (info_ptr != NULL) png_free_data(png_ptr, info_ptr, PNG_FREE_ALL, -1);
    if (png_ptr != NULL) png_destroy_write_struct(&png_ptr, (png_infopp)NULL);

    return code;
}

void gdal_img_float_to_rgb(const gladys::gdal& in, const std::string& file_out, int band_id) {
    const auto& band = in.bands[band_id];
    size_t width  = in.get_width();
    size_t height = in.get_height();
    size_t px_x, px_y;
    auto minmax = std::minmax_element(band.begin(), band.end());
    float min = *minmax.first;
    float max = *minmax.second;
    float diff = max - min;
    float hue;
    rgb_t rgb;
    if (diff == 0) // max == min (useless band)
        return;
    png_rows_t img(height);
    for (px_y = 0; px_y < height; px_y++) {
        img[px_y].resize(width * 3);
        for (px_x = 0; px_x < width; px_x++) {
            hue = (band[px_x + px_y * width] - min) / diff;
            if (hue > 1) hue = 1;
            rgb = float_to_rgb(hue);
            img[px_y][px_x * 3]     = rgb.r * 255;
            img[px_y][px_x * 3 + 1] = rgb.g * 255;
            img[px_y][px_x * 3 + 2] = rgb.b * 255;
        }
    }
    // save png
    png_text_t text;
    // save metadata in png header {'min':min,'max':max}
    text["minmax"] = "{'min':"+ std::to_string(min) +",'max':"+ std::to_string(max) +"}";
    save_img(file_out, img, width, height, text);
}

void gdal_float_to_rgb(const std::string& file_in, const std::string& file_out, int band_id=1) {
    gladys::gdal in(file_in);
    if (band_id < 0) {
        for (band_id = 0; band_id < in.bands.size(); band_id++) {
            std::string filename = file_out + std::to_string(band_id) + ".png";
            gdal_img_float_to_rgb(in, filename, band_id);
        }
    } else {
        gdal_img_float_to_rgb(in, file_out, band_id);
    }
}

int main(int argc, char * argv[])
{
    size_t band_id = 1;
    if (argc < 3) {
        std::cerr<<"usage: gdal_float_rgb file_in.tif file_out.png [band]"<<std::endl;
        return EXIT_FAILURE;
    }
    if (argc > 3) {
        band_id = std::stol(argv[3]);
    }
    gdal_float_to_rgb(argv[1], argv[2], band_id);
    return EXIT_SUCCESS;
}
