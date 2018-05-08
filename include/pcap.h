#ifndef PCAP_H
#define PCAP_H

/* https://wiki.wireshark.org/Development/LibpcapFileFormat */

struct PcapGlobalHeader {
	uint32_t magic_number;   // magic number
	uint16_t version_major;  // major version number
	uint16_t version_minor;  // minor version number
	int32_t thiszone;        // GMT to local correction
	uint32_t sigfigs;        // accuracy of timestamps
	uint32_t snaplen;        // max length of captured packets, in octets
	uint32_t network;        // data link type
};

struct PcapPacketHeader {
	uint32_t ts_sec;    // timestamp seconds
	uint32_t ts_usec;   // timestamp microseconds
	uint32_t incl_len;  // number of octets of packet saved in file
	uint32_t orig_len;  // actual length of packet
};

#if 0
/* for reference. if writing a pcap file, write the following header at beginning of file. 
   following settings work according to URL above.  tested with VeloView */
const PcapGlobalHeader kPcapHeader {
	magic_number: 0xa1b2c3d4,
	version_major: 2,
	version_minor: 4,
	thiszone: 0,
	sigfigs: 0,
	snaplen: 65535,
	network: 1,
};
#endif

#endif

