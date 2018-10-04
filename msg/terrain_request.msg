uint64 timestamp	# time since system start (microseconds)
int32 lat			# Latitude in 1E-7 degrees
int32 lon			# Longitude in 1E-7 degrees
uint16 grid_spacing # grid spacing in meters 
uint64 mask         # Bitmask of requested 4x4 grids (row major 8x7 array of grids, 56 bits)