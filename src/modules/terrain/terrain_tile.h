/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @file terrain_tile.h
 * Provides functions for handling the terrain tile
 *
 * @author Matej Frančeškin <matej.franceskin@gmail.com>
 */

#pragma once

#include <cinttypes>
#include <string>
#include <sstream>
#include <drivers/drv_hrt.h>
#include <platforms/px4_defines.h>

#define TILE_4X4_WIDTH 8
#define TILE_4X4_HEIGHT 7
#define TILES_4X4_NUM TILE_4X4_WIDTH * TILE_4X4_HEIGHT
#define MASK_ALL ((uint64_t)1 << (TILES_4X4_NUM + 1)) - 1

class TerrainTile
{
public:
    float lat;
    float lng;

	TerrainTile(int _lati, int _lngi, int _lat_offset, int _lng_offset);

    static int get_coord_deg(float coord)
    {
        return (coord < 0) ? (int)coord-1 : (int)coord;
    }

    int32_t get_lat()
    {
        return (int32_t)(1.0E7f * lat);
    }

    int32_t get_lng()
    {
        return (int32_t)(1.0E7f * lng);
    }

    bool valid()
    {
        return (mask == 0);
    }

    uint64_t get_mask()
    {
        return mask;
    }

    float get_elevation(int _lati, int _lngi, int _lat_offset, int _lng_offset);

    bool load();

    bool save();

    bool matches(int _lati, int _lngi, int _lat_offset, int _lng_offset)
    {
        return
            lati == _lati && lngi == _lngi && 
            lat_offset == _lat_offset - _lat_offset % matrix_width &&
            lng_offset == _lng_offset - _lng_offset % matrix_height;
    }

    bool set_terrain_data(uint8_t grid_bit, int16_t *data);

private:
    static const int matrix_width = TILE_4X4_WIDTH * 4;
    static const int matrix_height = TILE_4X4_HEIGHT * 4;

    int lati;
    int lngi;
    int lat_offset;
    int lng_offset;
    uint64_t mask;

    hrt_abstime access_timestamp;

	std::int16_t elevations[matrix_width][matrix_height];

    std::string get_file_name()
    {
        std::ostringstream oss;
        oss << PX4_STORAGEDIR"/terrain/" << lati << "_" << lat_offset << "-" << lngi << "_" << lng_offset;
        return oss.str();
    }
};
