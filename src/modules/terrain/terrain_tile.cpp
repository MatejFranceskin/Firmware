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
 * @file terrain_tile.cpp
 * Terrain elevation tile encapsulization.
 *
 * @author Matej Frančeškin <matej.franceskin@gmail.com>
 */

#include <stdio.h>
#include <fstream>
#include <limits>
#include "terrain_tile.h"

float
TerrainTile::get_elevation(int _latd, int _lond, int _lat_offset, int _lon_offset)
{
    int lato = _lat_offset - lat_offset;
    int lono = _lon_offset - lon_offset;

    if (valid() && matches(_latd, _lond, lato, lono))
    {
        access_timestamp = hrt_absolute_time();
        return elevations[lato][lono];
    }

    return std::numeric_limits<float>::quiet_NaN();
}

bool
TerrainTile::set_terrain_data(uint8_t grid_bit, int16_t *data)
{
    uint64_t b = ((uint64_t)1) << grid_bit;
    if ((mask & b) == 0 || grid_bit >= TILES_4X4_NUM)
        return false;
    mask &= ~b;
    PX4_INFO("set_terrain_data mask:%lx b:%lx", mask, b);
    uint8_t io = (grid_bit / 8) * 4;
    uint8_t jo = (grid_bit % 8) * 4;
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            elevations[io + i][jo + j] = data[i * 4 + j];

    return true;
}

bool
TerrainTile::load()
{
    std::ifstream infile;
    std::string fn = get_file_name();
    infile.open(fn, std::ios::binary | std::ios::in);
    if (infile.fail()) {
        PX4_DEBUG("Failed to open infile: %s", fn.c_str());
        return false;
    }
    infile.read((char*) elevations, sizeof(elevations));
    infile.close();
    mask = !infile.bad() ? 0 : MASK_ALL;

    return valid();
}

bool
TerrainTile::save()
{
    if (!valid())
        return false;
    std::ofstream outfile;
    std::string fn = get_file_name();
    outfile.open(fn, std::ios::binary | std::ios::out);
    if (outfile.fail()) {
        PX4_ERR("Failed to open outfile: %s", fn.c_str());
        return false;
    }
    outfile.write((char*)elevations, sizeof(elevations));
    outfile.close();

    return !outfile.bad();
}