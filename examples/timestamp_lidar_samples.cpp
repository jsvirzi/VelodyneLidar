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

size_t convertLidarDataPacketToPointCloud(const LidarDataPacket *lidar_packet, uint64_t data_packet_time, LidarData *point_cloud, size_t max_points) {
    int errors = 0;
#if 0
    float azimuth[kNumLidarBlocksInPacket];
        /* Find delta azimuth between blocks. In principle, one only needs to take the
         * difference between two blocks, but we will get a better estimate by
         * averaging over blocks in a packet.
         * TODO: Perhaps compute variance to see if this really makes any difference.
         */
        float rms_delta_azimuth = 0.0f;
        azimuth[0] = getAzimuth(lidar_packet.data_block[0]);
        for (size_t iblock = 1; iblock < kNumLidarBlocksInPacket; iblock++) {
            azimuth[iblock] = getAzimuth(lidar_packet.data_block[iblock]);
            // We divide by half because each block actually contains two sets of firing data
            float delta = 0.5 * (azimuth[iblock] - azimuth[iblock - 1]);
            if (delta < 0.0) delta += M_PI;
            rms_delta_azimuth += (delta * delta);
        }
        float delta_azimuth = sqrt(rms_delta_azimuth / (kNumLidarBlocksInPacket - 1));
#else
    float delta_azimuth = 0.00694f; /* empirically derived. why keep calculating the number? */
#endif

    const uint64_t top_of_hour_us = (data_packet_time / (60 * 60 * 1000) * (60 * 60 * 1000)) * 1000;
    const uint32_t time_since_top_of_hour_us = unalignedByteStreamToUint32(lidar_packet->timestamp);
    const uint64_t packet_begin_time_us = top_of_hour_us + time_since_top_of_hour_us;

    LidarData *lidar_data = point_cloud;

    bool first = true;
    uint64_t first_timestamp = 0;

    uint64_t datum_time_us = 0;
    size_t n_points = 0;
    for (size_t iblock = 0; iblock < kNumLidarBlocksInPacket; iblock++) {
        const LidarDataBlock &lidar_data_block = lidar_packet->data_block[iblock];
        float azimuth0 = getAzimuth(lidar_data_block);
        for (size_t ifiring = 0; ifiring != 2; ifiring++, azimuth0 += delta_azimuth) {
            for (size_t ichannel = 0; ichannel < kNumLidarChannels; ichannel++) {
                datum_time_us = packet_begin_time_us + kRelativeTimeFromBeginningOfPacket[n_points];
                const LidarChannelDatum &datum = lidar_data_block.data[kNumLidarChannels * ifiring + ichannel];
                lidar_data->time = datum_time_us; /* microseconds */
                lidar_data->channel = ichannel;
                lidar_data->reflectivity = datum.reflectivity;
                lidar_data->phi = azimuth0 + ichannel * kCycleTimeBetweenFirings * kLidarAngularVelocity;
                lidar_data->theta = kLaserPolarAngle[ichannel];
                lidar_data->R = kLidarDistanceUnit * unalignedByteStreamToUint16(datum.distance);

                if (first) {
                    printf("start timestamp = %" PRIu64 "\n", datum_time_us);
                    first_timestamp = datum_time_us;
                    first = false;
                }

                ++lidar_data;
                ++n_points;
                if (n_points == max_points) {
                    return n_points;
                }
            }
        }
    }

    printf("final timestamp = %" PRIu64 "\n", datum_time_us);
    printf("delta timestamp = %" PRIu64 "\n", datum_time_us - first_timestamp);
    getchar();

    return n_points;
}

int main(int argc, char **argv) {
	std::string ifile = "/home/jsvirzi/data/ublox-novatel-comparison/gps-testing/raw/lidar.pcap";
	for (int i = 1; i < argc; ++i) {
		if (strcmp(argv[i], "-i") == 0) {
			ifile = argv[++i];
		}
	}

	LidarDataPacket *lidar_data_packet;
	LidarPositionPacket *lidar_position_packet;
	void *p;

	VelodynePacketReader *velodynePacketReader = new VelodynePacketReader(ifile.c_str());

	const size_t max_points = 16384;
	LidarData *point_cloud = new LidarData [max_points];

	int type = velodynePacketReader->readPacket(&p);
    uint64_t gps_time = 0;
	while (type != VelodynePacketReader::LidarPacketTypes) {
		switch (type) {
		case VelodynePacketReader::TypeLidarDataPacket:
			lidar_data_packet = (LidarDataPacket *)p;
			convertLidarDataPacketToPointCloud(lidar_data_packet, gps_time, point_cloud, max_points);
			break;
		case VelodynePacketReader::TypeLidarPositionPacket:
		    lidar_position_packet = (LidarPositionPacket *)p;
            std::string nmea = (char *)lidar_position_packet->nmea_sentence;
            std::string gprmc = nmea.substr(0, nmea.find_first_of("\r"));
			int status = parseGprmc(gprmc, &gps_time, 0, 0);
			if (status == 0) {
			    printf("gps time = %" PRIu64 ". from NMEA = [%s]\n", gps_time, gprmc.c_str());
			} else {
			    printf("ERROR gps time. NMEA = [%s]\n", gprmc.c_str());
			}
			break;
		}
		type = velodynePacketReader->readPacket(&p);
	}
}

