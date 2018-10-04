uint64 timestamp	# time since system start (microseconds)
int32 lat			# Latitude in 1E-7 degrees
int32 lon			# Longitude in 1E-7 degrees
uint16 grid_spacing # grid spacing in meters 
uint8 grid_bit      # bit within the terrain request mask
int16[16] data      # Terrain data AMSL in meters