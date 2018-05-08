//
// Created by jsvirzi on 6/2/17.
//

#ifndef VELODYNE_PACKET_READER_H
#define VELODYNE_PACKET_READER_H

#include <stdint.h>

#include "pcap.h"
#include "velodyne.h"

class VelodynePacketReader {
	public:
	enum {
		TypeLidarDataPacket,
		TypeLidarPositionPacket,
		LidarPacketTypes,
	};
	VelodynePacketReader(const char *file);
	int readPacket(void **p);
	int fd;
#define NPOINTS (3600 * 16)
	LidarData pointCloud[NPOINTS];
	LidarPacketHeader lidarPacketHeader;
	LidarDataPacket lidarDataPacket;
	LidarPositionPacket lidarPositionPacket;
	PcapPacketHeader packetHeader;
};

#endif // PCAP_READER_H
