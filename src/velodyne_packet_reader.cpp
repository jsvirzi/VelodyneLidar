#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>

#include "velodyne_packet_reader.h"

VelodynePacketReader::VelodynePacketReader(const char *filename) {
	PcapGlobalHeader pcapGlobalHeader;
	fd = open(filename, O_RDONLY, S_IREAD);
	read(fd, &pcapGlobalHeader, sizeof(PcapGlobalHeader)); /* this is overhead at beginning of file */
}

int VelodynePacketReader::readPacket(void **p) {
	int type;
	memset(&packetHeader, 0, sizeof(packetHeader));
	memset(&lidarPacketHeader, 0, sizeof(lidarPacketHeader));
	int size = read(fd, &packetHeader, sizeof(packetHeader));
	size = packetHeader.orig_len;
	read(fd, &lidarPacketHeader, sizeof(LidarPacketHeader));
	if (size == (lidarDataPacketSize + sizeof(LidarPacketHeader))) {
		read(fd, &lidarDataPacket, lidarDataPacketSize);
		*p = (void *)&lidarDataPacket;
		type = TypeLidarDataPacket;
	} else if (size == (lidarPositionPacketSize + sizeof(LidarPacketHeader))) {
		read(fd, &lidarPositionPacket, lidarPositionPacketSize);
		*p = (void *)&lidarPositionPacket;
		type = TypeLidarPositionPacket;
	} else {
		*p = 0;
		type = LidarPacketTypes;
	}
	return type;
}

inline uint32_t unalignedByteStreamToUint32(const uint8_t *p) {
	uint32_t result0 = *p++;
	uint32_t result1 = *p++;
	uint32_t result2 = *p++;
	uint32_t result3 = *p;
	uint32_t result = result0 | (result1 << 8) | (result2 << 16) | (result3 << 24);
	return result;
}

uint64_t VelodynePacketReader::analyzePacketTimestamps(const uint32_t positionPacketTimestamp, const uint32_t dataPacketTimestamp,
	const uint64_t previousTimestamp, int &state) {
	const uint64_t topOfHourUs = (positionPacketTimestamp / (60 * 60 * 1000) * (60 * 60 * 1000)) * 1000;
	const uint64_t packetBeginTimeUs = topOfHourUs + dataPacketTimestamp;

	const unsigned int kMinPacketTime = 53;
	const unsigned int kMaxPacketTime = 57;

	if (packetBeginTimeUs < previousTimestamp) {
		state = LidarStateMonotonicityViolation;
	} else {
		uint64_t diffPacketTime = packetBeginTimeUs - previousTimestamp;
		bool packetTimeOk = (kMinPacketTime <= diffPacketTime) && (diffPacketTime <= kMaxPacketTime);
		state = packetTimeOk ? LidarStateGood : LidarStateTimingViolation;
	}
	return packetBeginTimeUs;
}

uint64_t VelodynePacketReader::analyzePointCloudTimestamps(const uint32_t positionPacketTimestamp, const LidarData *pointCloud, int nPoints,
	int &state) {

	switch (state) {
	case LidarStateUnknown: {
		break;
	}
	}
}

#if 0

	switch (state) {
	case LidarStateUnknown: {
		uint64_t diff_sample_time = packet_begin_time_us - previousTimestamp;
		++lidar_state_count;
		bool recharge_time_ok = (kMinRechargeTime <= diff_sample_time) && (diff_sample_time <= kMaxRechargeTime);
		if (recharge_time_ok) {
			fprintf(fp, "INFO(LIDAR): synchronized at t = %" PRIu64 ". state count = %d\n", lidar_sample_time, lidar_state_count);
			lidar_state = RECHARGE;
			lidar_state_count = 0;
			is_good_sample = true;
		} else if (lidar_state_count >= kFirings) {
			fprintf(fp, "ERROR(LIDAR): too many samples without sync\n");
			lidar_state_count = 0; /* reset */
		}
		break;
	}

	case RECHARGE: { /* this state only lasts one clock tick. either it's good or it's bad */
		uint64_t diff_sample_time = lidar_sample_time - previous_lidar_sample_time;
		if (diff_sample_time <= kMaxFiringTime) {
			lidar_state = FIRING;
			lidar_state_count = 1;
			is_good_sample = true;
			if (previous_lidar_packet_time != 0) {
				const uint64_t lidar_packet_time = lidar_sample_time;
				if (lidar_packet_time < previous_lidar_packet_time) {
					uint64_t diff_packet_time = previous_lidar_packet_time - lidar_packet_time;
					fprintf(fp, "ERROR(LIDAR): packet monotonicity violation %" PRIu64 " = %" PRIu64 " - %" PRIu64 "\n",
						diff_packet_time, previous_lidar_packet_time, lidar_packet_time);
				} else {
					uint64_t diff_packet_time = lidar_packet_time - previous_lidar_packet_time;
					bool packet_time_ok = (kMinPacketTime <= diff_packet_time) && (diff_packet_time <= kMaxPacketTime);
					if (packet_time_ok == true) {
						++consecutive_good_packets;
						// fprintf(fp, "INFO: time between packets = %" PRIu64 "\n", diff_packet_time);
					} else if (((kMinPacketTime - 2) <= diff_packet_time) &&
						(diff_packet_time <= (kMaxPacketTime + 2))) {
						++consecutive_good_packets;
						fprintf(fp, "WARNING(LIDAR): suspicious time between packets = %" PRIu64 ". %zu consecutive good packets\n",
							diff_packet_time, consecutive_good_packets);
					} else {
						fprintf(fp,
							"ERROR(LIDAR): time between packets = %" PRIu64 ". %zu consecutive good packets\n",
							diff_packet_time, consecutive_good_packets);
						consecutive_good_packets = 0;
					}
				}
			}
			previous_lidar_packet_time = lidar_sample_time;
		} else {
			fprintf(fp, "ERROR(LIDAR): recuperating after recharge?\n");
			lidar_state = UNKNOWN;
			lidar_state_count = 0;
		}
		break;
	}

	case FIRING: {
		if (previous_lidar_sample_time > lidar_sample_time) {
			uint64_t diff_sample_time = previous_lidar_sample_time - lidar_sample_time;
			fprintf(fp, "ERROR(LIDAR): sample time monotonicity error. diff = %" PRIu64 " = %" PRIu64 " - %" PRIu64 "\n",
				diff_sample_time, previous_lidar_sample_time, lidar_sample_time);
		} else {
			uint64_t diff_sample_time = lidar_sample_time - previous_lidar_sample_time;
			++lidar_state_count;
			bool recharge_time_ok =
				(kMinRechargeTime <= diff_sample_time) && (diff_sample_time <= kMaxRechargeTime);
			if ((lidar_state_count < kFirings) && (diff_sample_time <= kMaxFiringTime)) {
				is_good_sample = true;
			} else if ((lidar_state_count == kFirings) && (recharge_time_ok == true)) {
				lidar_state = RECHARGE;
				is_good_sample = true;
			} else if ((lidar_state_count == kFirings) && (recharge_time_ok == false)) {
				bool recharge_time_forgiveable = ((kMinRechargeTime - 2) <= diff_sample_time) &&
					(diff_sample_time <= (kMaxRechargeTime + 2));
				if (recharge_time_forgiveable == true) {
					fprintf(fp, "WARNING(LIDAR): suspicious recharge time %" PRIu64 ". %zu consecutive good samples\n",
						diff_sample_time, consecutive_good_samples);
					lidar_state = RECHARGE;
					is_good_sample = true;
				} else {
					fprintf(fp, "ERROR(LIDAR): bad recharge time %" PRIu64 ". %zu consecutive good samples\n",
						diff_sample_time, consecutive_good_samples);
					lidar_state = UNKNOWN;
					lidar_state_count = 0;
				}
			} else if (diff_sample_time > kMaxFiringTime) {
				fprintf(fp, "ERROR(LIDAR): bad firing time = %" PRIu64 ". %zu consecutive good samples\n",
					diff_sample_time, consecutive_good_samples);
				lidar_state = UNKNOWN;
				lidar_state_count = 0;
			} else {
				fprintf(fp, "ERROR(LIDAR): unknown state. dT = %" PRIu64 ". %zu consecutive good samples\n",
					diff_sample_time, consecutive_good_samples);
				lidar_state = UNKNOWN;
				lidar_state_count = 0;
			}
		}
		break;
	}
	}

	if (is_good_sample) { ++consecutive_good_samples; }
	else { consecutive_good_samples = 0; }

	previous_lidar_sample_time = lidar_sample_time;
}


}

#endif