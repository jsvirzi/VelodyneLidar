#ifndef VELODYNE_H
#define VELODYNE_H

#include <math.h>
#include <inttypes.h>

/* 
 * based on information from
 * git clone https://github.com/Kitware/VeloView.git
 * filename: VeloView/VelodyneHDL/vtkPacketFileWriter.cxx
 */

const int kNumLidarBlocksInPacket = 12;
const int kNumLidarChannels = 16;
const int lidarDataPacketSize = 1206;
const int lidarBlockSize = 100;
const int lidarDatumSize = 3;
const int lidarPositionPacketSize = 512;

/* assumes that Lidar is rotating at a fixed frequency of 20Hz */
const float kLidarAngularVelocity = 2.0 * M_PI * 20.0;
const float kLidarDistanceUnit = 0.002; // 2 millimeters.
const float kCycleTimeBetweenFirings = 2.304 / 1e6;  // seconds.

const float kLaserPolarAngle[kNumLidarChannels] = {
    -15.0 * M_PI / 180.0,   1.0 * M_PI / 180.0, -13.0 * M_PI / 180.0,
      3.0 * M_PI / 180.0, -11.0 * M_PI / 180.0,   5.0 * M_PI / 180.0,
     -9.0 * M_PI / 180.0,   7.0 * M_PI / 180.0,  -7.0 * M_PI / 180.0,
      9.0 * M_PI / 180.0,  -5.0 * M_PI / 180.0,  11.0 * M_PI / 180.0,
     -3.0 * M_PI / 180.0,  13.0 * M_PI / 180.0,  -1.0 * M_PI / 180.0,
     15.0 * M_PI / 180.0,
};

/* in each data packet, there are 12 * 2 * 55.296us = 1.327ms which comprises
 * 16 * 2.304us (firing time) + 18.432 (recharge time).
 * no point in calculating each time the same number. put into lookup table.
 * units are in microsecs. these numbers are added to the data packet timestamp
 * which is number of microsecs since top of the hour */
const double kRelativeTimeFromBeginningOfPacket[16 * 12 * 2] = {
       0.000,        2.304,        4.608,        6.912,        9.216,       11.520,       13.824,       16.128,
      18.432,       20.736,       23.040,       25.344,       27.648,       29.952,       32.256,       34.560,
      55.296,       57.600,       59.904,       62.208,       64.512,       66.816,       69.120,       71.424,
      73.728,       76.032,       78.336,       80.640,       82.944,       85.248,       87.552,       89.856,
     110.592,      112.896,      115.200,      117.504,      119.808,      122.112,      124.416,      126.720,
     129.024,      131.328,      133.632,      135.936,      138.240,      140.544,      142.848,      145.152,
     165.888,      168.192,      170.496,      172.800,      175.104,      177.408,      179.712,      182.016,
     184.320,      186.624,      188.928,      191.232,      193.536,      195.840,      198.144,      200.448,
     221.184,      223.488,      225.792,      228.096,      230.400,      232.704,      235.008,      237.312,
     239.616,      241.920,      244.224,      246.528,      248.832,      251.136,      253.440,      255.744,
     276.480,      278.784,      281.088,      283.392,      285.696,      288.000,      290.304,      292.608,
     294.912,      297.216,      299.520,      301.824,      304.128,      306.432,      308.736,      311.040,
     331.776,      334.080,      336.384,      338.688,      340.992,      343.296,      345.600,      347.904,
     350.208,      352.512,      354.816,      357.120,      359.424,      361.728,      364.032,      366.336,
     387.072,      389.376,      391.680,      393.984,      396.288,      398.592,      400.896,      403.200,
     405.504,      407.808,      410.112,      412.416,      414.720,      417.024,      419.328,      421.632,
     442.368,      444.672,      446.976,      449.280,      451.584,      453.888,      456.192,      458.496,
     460.800,      463.104,      465.408,      467.712,      470.016,      472.320,      474.624,      476.928,
     497.664,      499.968,      502.272,      504.576,      506.880,      509.184,      511.488,      513.792,
     516.096,      518.400,      520.704,      523.008,      525.312,      527.616,      529.920,      532.224,
     552.960,      555.264,      557.568,      559.872,      562.176,      564.480,      566.784,      569.088,
     571.392,      573.696,      576.000,      578.304,      580.608,      582.912,      585.216,      587.520,
     608.256,      610.560,      612.864,      615.168,      617.472,      619.776,      622.080,      624.384,
     626.688,      628.992,      631.296,      633.600,      635.904,      638.208,      640.512,      642.816,
     663.552,      665.856,      668.160,      670.464,      672.768,      675.072,      677.376,      679.680,
     681.984,      684.288,      686.592,      688.896,      691.200,      693.504,      695.808,      698.112,
     718.848,      721.152,      723.456,      725.760,      728.064,      730.368,      732.672,      734.976,
     737.280,      739.584,      741.888,      744.192,      746.496,      748.800,      751.104,      753.408,
     774.144,      776.448,      778.752,      781.056,      783.360,      785.664,      787.968,      790.272,
     792.576,      794.880,      797.184,      799.488,      801.792,      804.096,      806.400,      808.704,
     829.440,      831.744,      834.048,      836.352,      838.656,      840.960,      843.264,      845.568,
     847.872,      850.176,      852.480,      854.784,      857.088,      859.392,      861.696,      864.000,
     884.736,      887.040,      889.344,      891.648,      893.952,      896.256,      898.560,      900.864,
     903.168,      905.472,      907.776,      910.080,      912.384,      914.688,      916.992,      919.296,
     940.032,      942.336,      944.640,      946.944,      949.248,      951.552,      953.856,      956.160,
     958.464,      960.768,      963.072,      965.376,      967.680,      969.984,      972.288,      974.592,
     995.328,      997.632,      999.936,     1002.240,     1004.544,     1006.848,     1009.152,     1011.456,
    1013.760,     1016.064,     1018.368,     1020.672,     1022.976,     1025.280,     1027.584,     1029.888,
    1050.624,     1052.928,     1055.232,     1057.536,     1059.840,     1062.144,     1064.448,     1066.752,
    1069.056,     1071.360,     1073.664,     1075.968,     1078.272,     1080.576,     1082.880,     1085.184,
    1105.920,     1108.224,     1110.528,     1112.832,     1115.136,     1117.440,     1119.744,     1122.048,
    1124.352,     1126.656,     1128.960,     1131.264,     1133.568,     1135.872,     1138.176,     1140.480,
    1161.216,     1163.520,     1165.824,     1168.128,     1170.432,     1172.736,     1175.040,     1177.344,
    1179.648,     1181.952,     1184.256,     1186.560,     1188.864,     1191.168,     1193.472,     1195.776,
    1216.512,     1218.816,     1221.120,     1223.424,     1225.728,     1228.032,     1230.336,     1232.640,
    1234.944,     1237.248,     1239.552,     1241.856,     1244.160,     1246.464,     1248.768,     1251.072,
    1271.808,     1274.112,     1276.416,     1278.720,     1281.024,     1283.328,     1285.632,     1287.936,
    1290.240,     1292.544,     1294.848,     1297.152,     1299.456,     1301.760,     1304.064,     1306.368
};

/* lidar data and position packets as received over ethernet */ 
struct LidarChannelDatum {
    uint8_t distance[2];
    uint8_t reflectivity;
};

struct LidarDataBlock {
    uint16_t flag;
    uint8_t azimuth_lo, azimuth_hi;
    LidarChannelDatum data[32];
};

struct LidarDataPacket {
    // uint8 header[42]; /* it could be this if we got the whole packet via udp */
    LidarDataBlock data_block[kNumLidarBlocksInPacket];
    uint8_t timestamp[4];
    uint8_t factory_field[2];
};

struct LidarPositionPacket {
    // uint8 header[42]; /* it could be this if we got the whole packet */
    uint8_t unused1[198];
    uint8_t timestamp[4];
    uint8_t pps_status;
    uint8_t unused2[3];
    int8_t nmea_sentence[72];
    uint8_t unused3[234];
};

/* azimuth in range (-pi, pi). lidar reports values in degrees (0, 359.99) */
float getAzimuth(const LidarDataBlock &block);

// constexpr size_t MaxLidarDataPerRevolution = 754 * 32;

struct LidarData {
    double R, theta, phi;  /* R = distance, phi = azimuth, theta = altitude */
    int reflectivity, channel;
    uint64_t time;
};

struct LidarPacketHeader {
    unsigned char header[42];
};

struct PositionPacketHeader {
    unsigned char header[42];
};

/*
enum ReturnModes {
  StrongestReturn = 0x37,
  LastReturn = 0x38,
  DualReturn = 0x39
};
*/

#endif

