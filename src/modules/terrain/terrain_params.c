/**
 * Terrain Grid Spacing
 *
 * @min 30
 * @max 240
 * @unit m
 * @group Terrain
 */
PARAM_DEFINE_INT32(TER_GRID, 30);

/**
 * Timeout waiting for TERRAIN_DATA
 *
 * @min 100
 * @max 100000
 * @unit ms
 * @group Terrain
 */
PARAM_DEFINE_INT32(TER_TIMEOUT, 4000);

/**
 * Maximum number of tiles kept in memory
 *
 * @min 1
 * @max 100
 * @unit ms
 * @group Terrain
 */
PARAM_DEFINE_INT32(TER_MAX_MEM, 4);

