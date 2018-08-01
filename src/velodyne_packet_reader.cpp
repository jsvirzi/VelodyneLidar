#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <stdio.h>

#include "velodyne_packet_reader.h"

static inline uint32_t unalignedByteStreamToUint32(const uint8_t *p) {
	uint32_t result0 = *p++;
	uint32_t result1 = *p++;
	uint32_t result2 = *p++;
	uint32_t result3 = *p;
	uint32_t result = result0 | (result1 << 8) | (result2 << 16) | (result3 << 24);
	return result;
}

void VelodynePacketReader::setup() {
	debug = false;
	nRead = 0;
}

VelodynePacketReader::VelodynePacketReader(const char *filename) {
	PcapGlobalHeader pcapGlobalHeader;
	fd = open(filename, O_RDONLY, S_IREAD);
	read(fd, &pcapGlobalHeader, sizeof(PcapGlobalHeader)); /* this is overhead at beginning of file */
	setup();
}

VelodynePacketReader::VelodynePacketReader(const char *ipAddress, int dataPort, int positionPort) {

	if (dataPort == 0) {
		constexpr int defaultDataPort = 8309;
		dataPort = defaultDataPort;
	}

	if (positionPort == 0) {
		constexpr int defaultPosPort = 8308;
		positionPort = defaultPosPort;
	}

	setup();

	fd = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (fd == -1) {
		perror("socket");
		return;
	}

	fdPos = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (fdPos == -1) {
		perror("socket");
		return;
	}

	nfds = fd;
	if (nfds < fdPos) {
		nfds = fdPos;
	}
	nfds = nfds + 1; /* pselect() requirements */

	int optval;
	socklen_t optlen;

	optlen = sizeof(optval);
	setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &optval, optlen);

	optlen = sizeof(optval);
	setsockopt(fdPos, SOL_SOCKET, SO_REUSEADDR, &optval, optlen);

	struct sockaddr_in data_socket_address;
	struct sockaddr_in position_socket_address;
	data_socket_address.sin_family = AF_INET;         // host byte order
	position_socket_address.sin_family = AF_INET;     // host byte order
	data_socket_address.sin_port = htons(dataPort);  // port in network byte order
	position_socket_address.sin_port = htons(positionPort);
	if ((ipAddress == 0) || (strlen(ipAddress) == 0)) {
		data_socket_address.sin_addr.s_addr = INADDR_ANY;      // Automatically fill in my IP.
		position_socket_address.sin_addr.s_addr = INADDR_ANY;  // Automatically fill in my IP.
	} else {
		data_socket_address.sin_addr.s_addr = inet_addr(ipAddress);      // Specify IP address.
		position_socket_address.sin_addr.s_addr = inet_addr(ipAddress);  // Specify IP address.
	}

	auto stat = bind(fd, reinterpret_cast<struct sockaddr *>(&data_socket_address), sizeof(struct sockaddr));

	if (stat == -1) {
		perror("bind");
		return;
	}

	stat = bind(fdPos, reinterpret_cast<struct sockaddr *>(&position_socket_address), sizeof(struct sockaddr));

	if (stat == -1) {
		perror("bind");
		return;
	}
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

int VelodynePacketReader::readPocket(void **p) {
	fd_set rfds;
	struct timespec ts{ /* 1 second timeout to allow for interrupts to be processed */
		tv_sec : 1, tv_nsec : 0
	};
	FD_ZERO(&rfds);
	FD_SET(fd, &rfds);
	FD_SET(fdPos, &rfds);

	if (debug) {
		int nBytes = snprintf(logBuffer, logBufferSize, "entering pselect()");
		printf("%s\n", logBuffer);
	}

	const int fdsAvailable = pselect(nfds, &rfds, nullptr, nullptr, &ts, nullptr);
	if (fdsAvailable < 0) {
		perror("LIDAR pselect failed.");
		return -1;
	} else if (fdsAvailable == 0) {
		if (debug) {
			int nBytes = snprintf(logBuffer, logBufferSize, "lidar packets timed out!!!");
			printf("%s\n", logBuffer);
		}
	}

	if (debug) {
		int nBytes = snprintf(logBuffer, logBufferSize, "exitting pselect()");
		printf("%s\n", logBuffer);
	}

	int type = LidarPacketTypes;

	if (FD_ISSET(fd, &rfds)) { /* read lidar data packets */

		struct sockaddr_in sender_address;
		socklen_t sender_address_len = sizeof(sender_address);
		const ssize_t nBytesRead = recvfrom(fd,
			&lidarDataPacket, sizeof(LidarDataPacket), 0, (struct sockaddr *) &sender_address, &sender_address_len);
		nRead += nBytesRead;

		if (debug) {
			int nBytes = snprintf(logBuffer, logBufferSize,
				"received %ld bytes from port %d", nBytesRead, sender_address.sin_port);
			printf("%s\n", logBuffer);
		}

		*p = (nBytesRead == sizeof(LidarDataPacket)) ? (void *) &lidarDataPacket : 0;
		type = TypeLidarDataPacket;

//			convertLidarPacketToLidarSamples(packet, &lidar->event_storage_ring, lidar->time_from_data,
//				lidar->next_event_time, lidar->delta_event_time, status);

	} else if (FD_ISSET(fdPos, &rfds)) { /* read lidar position packets */

		struct sockaddr_in sender_address;
		socklen_t sender_address_len = sizeof(sender_address);
		const ssize_t nBytesRead = recvfrom(fdPos,
			&lidarPositionPacket, sizeof(LidarPositionPacket), 0, (struct sockaddr *) &sender_address, &sender_address_len);
		nRead += nBytesRead;

		if (debug) {
			int nBytes = snprintf(logBuffer, logBufferSize,
				"received %ld bytes from port %d", nBytesRead, sender_address.sin_port);
			printf("%s\n", logBuffer);
		}

		*p = (nBytesRead == sizeof(LidarPositionPacket)) ? (void *) &lidarPositionPacket : 0;
		type = TypeLidarPositionPacket;
	}
	return type;
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