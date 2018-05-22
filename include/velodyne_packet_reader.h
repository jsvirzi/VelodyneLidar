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
		TypeLidarDataPacket = 0,
		TypeLidarPositionPacket,
		LidarPacketTypes,
	};
	enum {
		LidarStateUnknown = 0,
		LidarStateFiring,
		LidarStateRecharging,
		LidarStateMonotonicityViolation,
		LidarStateGood,
		LidarStateTimingViolation,
		NumberOfLidarStates
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
	uint64_t analyzePacketTimestamps(const uint32_t positionPacketTimestamp, const uint32_t dataPacketTimestamp,
		const uint64_t previousTimestamp, int &state);
	uint64_t analyzePointCloudTimestamps(const uint32_t positionPacketTimestamp, const LidarData *pointCloud, int nPoints, int &state);
};

#endif // PCAP_READER_H
