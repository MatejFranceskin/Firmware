/****************************************************************************
 *
 *   Copyright (C) 2018 PX4 Development Team. All rights reserved.
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
 * @file terrain_tests.cpp
 *
 * @author Matej Frančeškin <matej.franceskin@gmail.com>
 */

#include <systemlib/err.h>
#include <unit_test.h>
#include <sys/stat.h>
#include "../terrain_tile.h"

extern "C" __EXPORT int terrain_tests_main(int argc, char *argv[]);
extern "C" __EXPORT bool terrain_test();

class TerrainTest : public UnitTest
{
public:
	TerrainTest() {}
	virtual ~TerrainTest() = default;

	virtual bool run_tests(void);

	// We don't want any of these
	TerrainTest(const TerrainTest &);

	TerrainTest &operator=(const TerrainTest &);

private:
	virtual void _init(void);

	virtual void _cleanup(void);

	bool _test_matches_1(void);

	bool _test_matches_2(void);

	bool _test_1(void);
};

/// @brief Called before every test.
void TerrainTest::_init()
{
}

/// @brief Called after every test.
void TerrainTest::_cleanup()
{
}

/// @brief Test for matches method
bool TerrainTest::_test_matches_1(void)
{
	TerrainTile t1(24, 18, 234, 123);
	TerrainTile t2(24, 18, 235, 124);
	TerrainTile t3(24, 18, 133, 124);
	TerrainTile t4(-12, -8, 133, 124);
	TerrainTile t5(-12, -8, 134, 125);

	return t1.matches(t2) && !t1.matches(t3) && !t1.matches(t3) && t4.matches(t5) && !t4.matches(t3);
}

/// @brief Test for matches method
bool TerrainTest::_test_matches_2(void)
{
	TerrainTile t1(24, 18, 234, 123);

	return t1.matches(24, 18, 21, 12) && !t1.matches(24, 18, -52, 12) && !t1.matches(24, 18, 152, 112);
}

bool TerrainTest::_test_1(void)
{
	const uint16_t grid_spacing = 30;
	TerrainTile t(45, 13, 123, 321);
	t.lat = 45;
	t.lon = 13;

	for (uint8_t bit = 0; bit < 56; bit++) {
		terrain_data_s td;
		td.lat = t.lat;
		td.lon = t.lon;
		td.grid_bit = bit;
		td.grid_spacing = grid_spacing;

		for (int16_t i = 0; i < 16; i++) {
			td.data[i] = i * 10;
		}

		t.set_terrain_data(td);
	}

	mkdir("terrain", S_IRWXU | S_IRWXG | S_IRWXO);
	t.save(grid_spacing);
	TerrainTile t2(45, 13, 123, 321);
	t2.load(grid_spacing);

	int elev = t.get_elevation(45, 13, 128, 323);
	int elev2 = t2.get_elevation(45, 13, 128, 323);

	return elev == 30 && elev == elev2;
}

/// @brief Runs all the unit tests
bool TerrainTest::run_tests(void)
{
	ut_run_test(_test_matches_1);
	ut_run_test(_test_matches_2);
	ut_run_test(_test_1);

	return (_tests_failed == 0);
}

int terrain_tests_main(int argc, char *argv[])
{
	return terrain_test() ? 0 : -1;
}

ut_declare_test(terrain_test, TerrainTest)