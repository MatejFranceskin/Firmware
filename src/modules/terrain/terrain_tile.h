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

#include <drivers/drv_hrt.h>
#include <platforms/px4_defines.h>
#include <cinttypes>
#include <sstream>
#include <string>
#include <uORB/topics/terrain_data.h>

#define TILE_4X4_WIDTH 8
#define TILE_4X4_HEIGHT 7
#define TILES_4X4_NUM TILE_4X4_WIDTH *TILE_4X4_HEIGHT
#define MASK_ALL ((uint64_t)1 << TILES_4X4_NUM) - 1
#define TERRAIN_FOLDER PX4_STORAGEDIR "/terrain/"

class TerrainTile
{
public:
	int32_t lat;
	int32_t lon;
	int lat_offset;
	int lon_offset;

	TerrainTile(int _latd, int _lond, int _lat_offset, int _lon_offset)
	{
		latd = _latd;
		lond = _lond;
		lat_offset = round_down(_lat_offset, matrix_height);
		lon_offset = round_down(_lon_offset, matrix_width);
		mask = MASK_ALL;
	}

	static int get_coord_deg(float coord)
	{
		return (coord < 0) ? (int)coord - 1 : (int)coord;
	}

	bool valid() { return (mask == 0); }

	uint64_t get_mask() { return mask; }

	float get_elevation(int _latd, int _lond, int _lat_offset, int _lon_offset);

	bool load(int32_t grid_spacing);

	bool save(int32_t grid_spacing);

	bool matches(TerrainTile &t)
	{
		return latd == t.latd && lond == t.lond && lat_offset == t.lat_offset && lon_offset == t.lon_offset;
	}

	bool matches(int _latd, int _lond, int _lat_offset_diff, int _lon_offset_diff)
	{
		return	latd == _latd && lond == _lond && _lat_offset_diff >= 0 && _lat_offset_diff < matrix_height
			&& _lon_offset_diff >= 0 && _lon_offset_diff < matrix_width;
	}

	bool set_terrain_data(terrain_data_s &td);

private:
	static const int matrix_width = TILE_4X4_WIDTH * 4;
	static const int matrix_height = TILE_4X4_HEIGHT * 4;

	int latd;
	int lond;
	uint64_t mask;

	hrt_abstime access_timestamp;

	std::int16_t elevations[matrix_height][matrix_width];

	std::string get_file_name(int32_t grid_spacing)
	{
		std::ostringstream oss;
		oss << TERRAIN_FOLDER << grid_spacing << "_" << latd << "_" << lat_offset << "-" << lond << "_" << lon_offset;
		return oss.str();
	}

	int round_down(int num, int multiple)
	{
		int is_negative = (num < 0);
		return ((num - is_negative * (multiple - 1)) / multiple) * multiple;
	}

	void print_elevations()
	{
		for (int i = 0; i < matrix_height; i++) {
			std::ostringstream oss;
			oss << "elevations[" << i << "] = [";

			for (int j = 0; j < matrix_width; j++) {
				if (j > 0) {
					oss << ",";
				}

				oss << elevations[i][j];
			}

			oss << "]";
			PX4_INFO("%s", oss.str().c_str());
		}
	}
};
