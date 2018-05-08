all: lib/libvelodyne_packet_reader.so

lib/libvelodyne_packet_reader.so : src/velodyne_packet_reader.cpp include/velodyne_packet_reader.h include/pcap.h
	mkdir -p lib
	gcc -c -Iinclude -Wall -Werror -fpic src/velodyne_packet_reader.cpp 
	gcc -shared -o lib/libvelodyne_packet_reader.so velodyne_packet_reader.o
	rm velodyne_packet_reader.o

