uint64 timestamp	    # time since system start (microseconds)
int32 lat			    # Latitude in 1E-7 degrees
int32 lon			    # Longitude in 1E-7 degrees
uint16 spacing          # grid spacing in meters (zero if terrain at this location unavailable)
float32 terrain_height  # Terrain height AMSL in meters
float32 current_height  # Current vehicle height above lat/lon terrain height in meters
uint16 pending          # Number of 4x4 terrain blocks waiting to be received or read from disk
uint16 loaded           # Number of 4x4 terrain blocks in memory