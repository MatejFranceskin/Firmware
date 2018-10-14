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
 * @file terrain_main.cpp
 * Terrain elevation module.
 *
 * @author Matej Frančeškin <matej.franceskin@gmail.com>
 */
#include <drivers/drv_hrt.h>
#include <lib/ecl/geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <lib/ecl/geo/geo.h>
#include <sys/stat.h>
#include <matrix/math.hpp>
#include <parameters/param.h>
#include <perf/perf_counter.h>
#include <px4_module.h>
#include <px4_module_params.h>
#include <px4_workqueue.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/terrain_check.h>
#include <uORB/topics/terrain_data.h>
#include <uORB/topics/terrain_report.h>
#include <uORB/topics/terrain_request.h>

#include <limits.h>
#include <limits>
#include <list>
#include <mutex>

#include "terrain_tile.h"

using namespace time_literals;

#define SCHEDULE_INTERVAL	100000	/**< The schedule interval in usec (10 Hz) */

class TerrainModule : public ModuleBase<TerrainModule>, public ModuleParams
{
public:
	TerrainModule();

	~TerrainModule();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	// run the main loop
	void cycle();

	int print_status() override;

private:
	static struct work_s _work;

	orb_advert_t _terrain_request_pub{nullptr};			/**< terrain request topic */
	orb_advert_t _terrain_report_pub{nullptr};			/**< terrain report topic */

	int _vehicle_global_position_sub{-1};
	int _param_sub{-1};
	int _terrain_data_sub{-1};
	int _terrain_check_sub{-1};

	perf_counter_t _perf_elapsed{};
	perf_counter_t _perf_interval{};

	int	_instance{-1};

    float current_alt{std::numeric_limits<float>::quiet_NaN()};
    float current_terrain_alt{std::numeric_limits<float>::quiet_NaN()};
    uint16_t pending_tiles{0};
    uint16_t loaded_tiles{0};
    float lat_grid_spacing_in_degrees;
    float lon_grid_spacing_in_degrees;
    int lat_grid_spacing_origin;
    int lon_grid_spacing_origin;
    std::list<TerrainTile> tiles;
    std::list<TerrainTile> tiles_to_load;
    hrt_abstime data_timestamp;
   	std::mutex terrain_mutex;

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::TER_GRID>) terrain_grid_spacing,
		(ParamInt<px4::params::TER_TIMEOUT>) terrain_timeout
	)

	static void	cycle_trampoline(void *arg);

	int start();

	void update_params();

	bool subscribe_topics();

	float get_elevation(float lat, float lon);

	float get_elevation(double lat, double lon)
    {
        return get_elevation((float)lat, (float)lon);
    }

    float get_elevation(int32_t lat, int32_t lon)
    {
        return get_elevation(lat/1e7f, lon/1e7f);
    }

    void process_tiles_to_load();

    void process_terrain_data(terrain_data_s &);

    void process_terrain_check(terrain_check_s &);
};

work_s TerrainModule::_work = {};

TerrainModule::TerrainModule():
	ModuleParams(nullptr)
{
	_vehicle_global_position_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	_param_sub = orb_subscribe(ORB_ID(parameter_update));
	_terrain_data_sub = orb_subscribe(ORB_ID(terrain_data));
	_terrain_check_sub = orb_subscribe(ORB_ID(terrain_check));

	// initialise parameters
	update_params();

	_perf_elapsed = perf_alloc_once(PC_ELAPSED, "terrain elapsed");
	_perf_interval = perf_alloc_once(PC_INTERVAL, "terrain interval");

    lat_grid_spacing_origin = lon_grid_spacing_origin = INT_MAX;

    mkdir(TERRAIN_FOLDER, S_IRWXU | S_IRWXG | S_IRWXO);

    data_timestamp = hrt_absolute_time();    
}

TerrainModule::~TerrainModule()
{
	orb_unsubscribe(_vehicle_global_position_sub);
	orb_unsubscribe(_param_sub);
	orb_unsubscribe(_terrain_data_sub);
	orb_unsubscribe(_terrain_check_sub);

	orb_unadvertise(_terrain_request_pub);
	orb_unadvertise(_terrain_report_pub);

	perf_free(_perf_elapsed);
	perf_free(_perf_interval);
}

int
TerrainModule::task_spawn(int argc, char *argv[])
{
	/* schedule a cycle to start things */
	work_queue(LPWORK, &_work, (worker_t)&TerrainModule::cycle_trampoline, nullptr, 0);

	// wait until task is up & running
	if (wait_until_running() < 0) {
		_task_id = -1;

	} else {
		_task_id = task_id_is_work_queue;
		return PX4_OK;
	}

	return PX4_ERROR;
}

void
TerrainModule::cycle_trampoline(void *arg)
{
	TerrainModule *dev = reinterpret_cast<TerrainModule *>(arg);

	// check if the trampoline is called for the first time
	if (!dev) {
		dev = new TerrainModule();

		if (!dev) {
			PX4_ERR("alloc failed");
			return;
		}

		_object = dev;
	}

	if (dev) {
		dev->cycle();
	}
}

void
TerrainModule::cycle()
{
	perf_count(_perf_interval);
	perf_begin(_perf_elapsed);

	bool param_updated;
	orb_check(_param_sub, &param_updated);

	if (param_updated)
    {
		update_params();
	}

    bool terrain_data_received;
    orb_check(_terrain_data_sub, &terrain_data_received);
    if (terrain_data_received)
    {
    	terrain_data_s td = {};
        if (orb_copy(ORB_ID(terrain_data), _terrain_data_sub, &td) == PX4_OK)
            process_terrain_data(td);
        data_timestamp = hrt_absolute_time();
    }

    process_tiles_to_load();

	vehicle_global_position_s gpos = {};
	if (orb_copy(ORB_ID(vehicle_global_position), _vehicle_global_position_sub, &gpos) == PX4_OK)
    {
        current_alt = gpos.alt;
        current_terrain_alt = get_elevation(gpos.lat, gpos.lon);
	}

    bool terrain_check_received;
    orb_check(_terrain_check_sub, &terrain_check_received);
    if (terrain_check_received)
    {
    	terrain_check_s tc = {};
        if (orb_copy(ORB_ID(terrain_check), _terrain_check_sub, &tc) == PX4_OK)
            process_terrain_check(tc);
    }

	perf_end(_perf_elapsed);

	if (should_exit())
    {
		exit_and_cleanup();
	}
    else
    {
		/* schedule next cycle */
		work_queue(LPWORK, &_work, (worker_t)&TerrainModule::cycle_trampoline, this, USEC2TICK(SCHEDULE_INTERVAL));
	}
}

void TerrainModule::update_params()
{
	updateParams();
}

int TerrainModule::custom_command(int argc, char *argv[])
{
	if (!is_running())
    {
		int ret = TerrainModule::task_spawn(argc, argv);

		if (ret)
        {
			return ret;
		}
	}

	return print_usage("unknown command");
}

int TerrainModule::print_usage(const char *reason)
{
	if (reason)
    {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module maintains terrain database.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("terrain", "terrain");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int TerrainModule::print_status()
{
	perf_print_counter(_perf_elapsed);
	perf_print_counter(_perf_interval);

	if (_instance > -1)
	{
        PX4_INFO("Running.");
	}
    else
    {
		PX4_INFO("Running, but never published");
	}

	return 0;
}

extern "C" __EXPORT int terrain_main(int argc, char *argv[]);

int
terrain_main(int argc, char *argv[])
{
	return TerrainModule::main(argc, argv);
}

float
TerrainModule::get_elevation(float lat, float lon)
{
    int latd = TerrainTile::get_coord_deg(lat);
    int lond = TerrainTile::get_coord_deg(lon);

    std::lock_guard<std::mutex> lock(terrain_mutex);

    if (lat_grid_spacing_origin != latd || lon_grid_spacing_origin != lond)
    {
        lat_grid_spacing_origin = latd;
        lon_grid_spacing_origin = lond;
        double lat_t, lon_t;
        waypoint_from_heading_and_distance(latd, lond, math::radians(0.0f), terrain_grid_spacing.get(), &lat_t, &lon_t);
        lat_grid_spacing_in_degrees = (float)(lat_t - latd);
        waypoint_from_heading_and_distance(latd, lond, math::radians(90.0f), terrain_grid_spacing.get(), &lat_t, &lon_t);
        lon_grid_spacing_in_degrees = (float)(lon_t - lond);
    }

    int lat_offset = (int)(std::abs(lat - latd) / lat_grid_spacing_in_degrees);
    int lon_offset = (int)(std::abs(lon - lond) / lon_grid_spacing_in_degrees);

    for (auto i : tiles)
    {
        float res = i.get_elevation(latd, lond, lat_offset, lon_offset);
        if (!std::isnan(res))
            return res;
    }

    TerrainTile new_tile(latd, lond, lat_offset, lon_offset);

    for (auto i : tiles_to_load)
    {
        // Tile is still loading - we don't have the result yet
        if (i.matches(new_tile))
            return std::numeric_limits<float>::quiet_NaN();
    }

    // Check if we can load the tile from a local file
    if (new_tile.load())
    {
        tiles.push_back(new_tile);
        loaded_tiles += TILES_4X4_NUM;
        return new_tile.get_elevation(latd, lond, lat_offset, lon_offset);
    }

    new_tile.lat = (int32_t)(1.0E7f * (latd + new_tile.lat_offset * lat_grid_spacing_in_degrees));
    new_tile.lon = (int32_t)(1.0E7f * (lond + new_tile.lon_offset * lon_grid_spacing_in_degrees));

    pending_tiles += TILES_4X4_NUM;
    PX4_INFO("New tile to load: %d,%d", new_tile.lat, new_tile.lon);
    tiles_to_load.push_back(new_tile);
    
    return std::numeric_limits<float>::quiet_NaN();
}

void
TerrainModule::process_tiles_to_load()
{
    std::lock_guard<std::mutex> lock(terrain_mutex);

    const hrt_abstime time_now_usec = hrt_absolute_time();
    if ((time_now_usec - data_timestamp) < (hrt_abstime)(terrain_timeout.get() * 1000) || tiles_to_load.empty())
        return;
    data_timestamp = time_now_usec;
    TerrainTile &tile = tiles_to_load.front();

    terrain_request_s terrain_request_msg = {};

    terrain_request_msg.lat = tile.lat;
    terrain_request_msg.lon = tile.lon;
    terrain_request_msg.grid_spacing = (uint16_t)terrain_grid_spacing.get();
    terrain_request_msg.mask = tile.get_mask();

    PX4_INFO("New terrain request: %d,%d", terrain_request_msg.lat, terrain_request_msg.lon);

    orb_publish_auto(ORB_ID(terrain_request), &_terrain_request_pub, &terrain_request_msg, &_instance, ORB_PRIO_DEFAULT);
}

void
TerrainModule::process_terrain_data(terrain_data_s &td)
{
    PX4_INFO("Process terrain data (%d,%d) bit: %d", td.lat, td.lon, td.grid_bit);
    std::lock_guard<std::mutex> lock(terrain_mutex);
    if (tiles_to_load.empty())
        return;
    TerrainTile &tile = tiles_to_load.front();
    if (tile.set_terrain_data(td.grid_bit, td.data))
    {
        pending_tiles--;
        loaded_tiles++;
        if (tile.valid())
        {
            PX4_INFO("Terrain tile (%d,%d) saved", td.lat, td.lon);
            tile.save();
            tiles.push_back(tile);
            tiles_to_load.pop_front();
        }
    }
}

void
TerrainModule::process_terrain_check(terrain_check_s &tc)
{
    terrain_report_s terrain_report_msg = {};

    terrain_report_msg.lat = tc.lat;
    terrain_report_msg.lon = tc.lon;
    terrain_report_msg.spacing = (uint16_t)terrain_grid_spacing.get();
    terrain_report_msg.terrain_height = get_elevation(tc.lat, tc.lon);
    terrain_report_msg.current_height = current_alt;
    terrain_report_msg.pending = pending_tiles;
    terrain_report_msg.loaded = loaded_tiles;

    orb_publish_auto(ORB_ID(terrain_report), &_terrain_report_pub, &terrain_report_msg, &_instance, ORB_PRIO_DEFAULT);
}