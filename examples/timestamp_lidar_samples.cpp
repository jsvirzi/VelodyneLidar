#include <stdlib.h>
#include <string>
#include <string.h>

#include "velodyne_packet_reader.h"
#include "gps_utils.h"

static const uint16_t max_lidar_azimuth_reading = 35999;
static const float azimuth_degrees_x100_to_radians = M_PI / 18000.0;

inline uint16_t unalignedByteStreamToUint16(const uint8_t *p) {
    uint16_t result0 = *p++;
    uint16_t result1 = *p;
    uint16_t result = result0 | (result1 << 8);
    return result;
}

inline uint32_t unalignedByteStreamToUint32(const uint8_t *p) {
    uint32_t result0 = *p++;
    uint32_t result1 = *p++;
    uint32_t result2 = *p++;
    uint32_t result3 = *p;
    uint32_t result = result0 | (result1 << 8) | (result2 << 16) | (result3 << 24);
    return result;
}

inline float getAzimuth(const LidarDataBlock &block) {
    const uint8_t azimuth_bytes[] = {block.azimuth_lo, block.azimuth_hi};
    uint16_t azimuth_degrees_x100 = unalignedByteStreamToUint16(azimuth_bytes); /* TODO */
    if (azimuth_degrees_x100 > max_lidar_azimuth_reading) {
        azimuth_degrees_x100 = max_lidar_azimuth_reading; /* this is an error condition. TODO */
    }
    float radians = azimuth_degrees_x100 * azimuth_degrees_x100_to_radians;
    if (radians > M_PI) { radians -= 2 * M_PI; } /* bring into range */
    return radians;
}

size_t convertLidarDataPacketToPointCloud(const LidarDataPacket *dataPacket, uint64_t dataPacketTime, LidarData *pointCloud, size_t maxPoints) {
    int errors = 0;
    float azimuth[kNumLidarBlocksInPacket];

    /* Find delta azimuth between blocks. In principle, one only needs to take the
     * difference between two blocks, but we will get a better estimate by
     * averaging over blocks in a packet.
     * TODO: Perhaps compute variance to see if this really makes any difference.
     */
    float rms_delta_azimuth = 0.0f;
    azimuth[0] = getAzimuth(dataPacket->data_block[0]);
    for (size_t iblock = 1; iblock < kNumLidarBlocksInPacket; iblock++) {
        azimuth[iblock] = getAzimuth(dataPacket->data_block[iblock]);
        // We divide by half because each block actually contains two sets of firing data
        float delta = 0.5 * (azimuth[iblock] - azimuth[iblock - 1]);
        if (delta < 0.0) delta += M_PI;
        rms_delta_azimuth += (delta * delta);
    }
    float delta_azimuth = sqrt(rms_delta_azimuth / (kNumLidarBlocksInPacket - 1));

    const uint64_t topOfHourUs = (dataPacketTime / (60 * 60 * 1000) * (60 * 60 * 1000)) * 1000;
    const uint32_t timeSinceTopOfHourUs = unalignedByteStreamToUint32(dataPacket->timestamp);
    const uint64_t packetBeginTimeUs = topOfHourUs + timeSinceTopOfHourUs;

    LidarData *lidar_data = pointCloud;

    uint64_t datumTimeUs = 0;
    size_t nPoints = 0;
    for (size_t iblock = 0; iblock < kNumLidarBlocksInPacket; iblock++) {
        const LidarDataBlock &lidar_data_block = dataPacket->data_block[iblock];
        float azimuth0 = getAzimuth(lidar_data_block);
        for (size_t ifiring = 0; ifiring != 2; ifiring++, azimuth0 += delta_azimuth) {
            for (size_t ichannel = 0; ichannel < kNumLidarChannels; ichannel++) {
                datumTimeUs = packetBeginTimeUs + kRelativeTimeFromBeginningOfPacket[nPoints];
                const LidarChannelDatum &datum = lidar_data_block.data[kNumLidarChannels * ifiring + ichannel];
                lidar_data->time = datumTimeUs; /* microseconds */
                lidar_data->channel = ichannel;
                lidar_data->reflectivity = datum.reflectivity;
                lidar_data->phi = azimuth0 + ichannel * kCycleTimeBetweenFirings * kLidarAngularVelocity;
                lidar_data->theta = kLaserPolarAngle[ichannel];
                lidar_data->R = kLidarDistanceUnit * unalignedByteStreamToUint16(datum.distance);

                ++lidar_data;
                ++nPoints;
                if (nPoints == maxPoints) {
                    return nPoints;
                }
            }
        }
    }

    return nPoints;
}

int main(int argc, char **argv) {
	std::string ifile = "/home/jsvirzi/data/ublox-novatel-comparison/gps-testing/raw/lidar.pcap";
	for (int i = 1; i < argc; ++i) {
		if (strcmp(argv[i], "-i") == 0) {
			ifile = argv[++i];
		}
	}

	LidarDataPacket *lidarDataPacket;
	LidarPositionPacket *lidarPositionPacket;
	void *p;

	VelodynePacketReader *velodynePacketReader = new VelodynePacketReader(ifile.c_str());

	const size_t max_points = 16384;
	LidarData *point_cloud = new LidarData [max_points];

	int type = velodynePacketReader->readPacket(&p);
    uint64_t gpsTime = 0;
	while (type != VelodynePacketReader::LidarPacketTypes) {
		switch (type) {
		case VelodynePacketReader::TypeLidarDataPacket:
			lidarDataPacket = (LidarDataPacket *)p;
			convertLidarDataPacketToPointCloud(lidarDataPacket, gpsTime, point_cloud, max_points);
			break;
		case VelodynePacketReader::TypeLidarPositionPacket:
		    lidarPositionPacket = (LidarPositionPacket *)p;
            std::string nmea = (char *)lidarPositionPacket->nmea_sentence;
            std::string gprmc = nmea.substr(0, nmea.find_first_of("\r"));
			int status = parseGprmc(gprmc, &gpsTime, 0, 0);
			if (status == 0) {
			    printf("gps time = %" PRIu64 ". from NMEA = [%s]\n", gpsTime, gprmc.c_str());
			} else {
			    printf("ERROR gps time. NMEA = [%s]\n", gprmc.c_str());
			}
			break;
		}
		type = velodynePacketReader->readPacket(&p);
	}
}

