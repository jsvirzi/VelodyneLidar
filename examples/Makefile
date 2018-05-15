all: timestamp_lidar_samples

INCS=-I${HOME}/projects/VelodyneLidar/include -I${HOME}/projects/GpsUtils/include
LIBS=-L${HOME}/projects/VelodyneLidar/lib -lvelodyne_packet_reader -L${HOME}/projects/GpsUtils/lib -lgps_utils

timestamp_lidar_samples : timestamp_lidar_samples.cpp
	gcc timestamp_lidar_samples.cpp ${INCS} ${LIBS}
