#include <gtest/gtest.h>
#include "source_driver_ros2.hpp"
#include "udp_protocol_v1_4.h"
#include "udp1_4_parser.h"
#ifdef USE_CUDA
#include "udp1_4_parser_gpu.h"
#endif

namespace {
class ConversionDriver : public SourceDriver {
public:
  ConversionDriver() : SourceDriver(DATA_FROM_ROS_PACKET) {frame_id_ = "hesai_lidar";}
  using SourceDriver::ToRosMsg;
  void compact() {output_filter_.enabled = true; output_filter_.stride = 2;}
};

template<class T>
T field(const sensor_msgs::msg::PointCloud2 & cloud, size_t point, size_t offset) {
  T value;
  std::memcpy(&value, cloud.data.data() + point * cloud.point_step + offset, sizeof(T));
  return value;
}
}

TEST(DriverConversion, Jt128PacketAccelerationAndGyroRemainInSIUnits) {
  // Protocol 1.4's actual SDK accessors, not fabricated SI-valued SDK output.
  hesai::lidar::HS_LIDAR_TAIL_IMU_ME_V4 packet{};
  packet.m_u16IMUAccelUnit = 1000;  // 0.001 g per count
  packet.m_u16IMUAngVelUnit = 100; // 0.001 degrees/s per count
  packet.m_i16IMUXAccel = 1000;
  packet.m_i16IMUYAccel = -2000;
  packet.m_i16IMUZAccel = 500;
  packet.m_i16IMUXAngVel = 10000;
  packet.m_i16IMUYAngVel = -20000;
  packet.m_i16IMUZAngVel = 30000;
  LidarImuData decoded;
  decoded.timestamp = 1700000000.125;
  decoded.imu_accel_x = packet.GetIMUXAccel();
  decoded.imu_accel_y = packet.GetIMUYAccel();
  decoded.imu_accel_z = packet.GetIMUZAccel();
  decoded.imu_ang_vel_x = packet.GetIMUXAngVel();
  decoded.imu_ang_vel_y = packet.GetIMUYAngVel();
  decoded.imu_ang_vel_z = packet.GetIMUZAngVel();
  ConversionDriver driver;
  const auto msg = driver.ToRosMsg(decoded);
  EXPECT_DOUBLE_EQ(msg.linear_acceleration.x, 9.80665);
  EXPECT_DOUBLE_EQ(msg.linear_acceleration.y, -19.6133);
  EXPECT_DOUBLE_EQ(msg.linear_acceleration.z, 4.903325);
  EXPECT_NEAR(msg.angular_velocity.x, M_PI / 18, 1e-12);
  EXPECT_NEAR(msg.angular_velocity.y, -M_PI / 9, 1e-12);
  EXPECT_NEAR(msg.angular_velocity.z, M_PI / 6, 1e-12);
  EXPECT_EQ(msg.header.frame_id, "hesai_lidar");
  EXPECT_EQ(msg.header.stamp.sec, 1700000000);
  EXPECT_EQ(msg.header.stamp.nanosec, 125000000U);
}

TEST(DriverConversion, SDKFrameUsesCompactOutputAndRetainsTheLastFiringTime) {
  LidarDecodedFrame<LidarPointXYZIRT> frame(1, 8);
  frame.points_num = 6;
  frame.frame_start_timestamp = 10;
  for (size_t i = 0; i < frame.points_num; ++i) {
    auto & p = frame.points[i];
    p.x = 2; p.y = 0; p.z = 1;
    p.intensity = float(i); p.ring = uint16_t(i);
    p.timestamp = 10 + 0.01 * i;
  }
  ConversionDriver driver;
  driver.compact();
  const auto cloud = driver.ToRosMsg(frame, "hesai_lidar");
  ASSERT_EQ(cloud.width, 4U);
  EXPECT_EQ(cloud.point_step, 28U);
  EXPECT_EQ(field<uint16_t>(cloud, 2, 16), 4);
  EXPECT_DOUBLE_EQ(field<double>(cloud, 2, 18), 10.04);
  EXPECT_EQ(field<uint16_t>(cloud, 2, 26), 2);
  EXPECT_TRUE(std::isnan(field<float>(cloud, 3, 0)));
  EXPECT_DOUBLE_EQ(field<double>(cloud, 3, 18), 10.05);
}

TEST(DriverConversion, UpstreamVariableLengthPtpPayloadIsPreserved) {
  ConversionDriver driver;
  u8Array_t payload(64);
  for (size_t i = 0; i < payload.size(); ++i) payload[i] = uint8_t(i);
  const auto msg = driver.ToRosMsg(uint8_t(3), payload);
  EXPECT_EQ(msg.ptp_lock_offset, 3);
  EXPECT_EQ(msg.ptp_status_size, payload.size());
  EXPECT_EQ(msg.ptp_status, payload);
}

namespace {
using namespace hesai::lidar;

// One protocol 1.4 packet with 128 channels, two blocks and an IMU tail.
// No sensor connection or PTC command is involved in these tests.
UdpPacket jt128Packet(uint64_t receive_time) {
  std::vector<uint8_t> bytes;
  auto append = [&bytes](const auto & value) {
    const auto * begin = reinterpret_cast<const uint8_t *>(&value);
    bytes.insert(bytes.end(), begin, begin + sizeof(value));
  };
  HS_LIDAR_PRE_HEADER pre{};
  pre.m_u16Delimiter = 0xffee;
  pre.m_u8VersionMajor = 1;
  pre.m_u8VersionMinor = 4;
  append(pre);
  HS_LIDAR_HEADER_ME_V4 header{};
  header.m_u8LaserNum = 128;
  header.m_u8BlockNum = 2;
  header.m_u8EchoCount = header.m_u8EchoNum = 1;
  header.m_u8DistUnit = 4;
  header.m_u8Status = HS_LIDAR_HEADER_ME_V4::kIMU;
  append(header);
  for (int block = 0; block < 2; ++block) {
    HS_LIDAR_BODY_AZIMUTH_ME_V4 azimuth{};
    azimuth.m_u16Azimuth = block == 0 ? 0 : 9000;
    append(azimuth);
    for (int channel = 0; channel < 128; ++channel) {
      HS_LIDAR_BODY_CHN_UNIT_NO_CONF_ME_V4 point{};
      point.m_u16Distance = 500;  // 2 metres
      point.m_u8Reflectivity = channel;
      append(point);
    }
  }
  append(HS_LIDAR_BODY_CRC_ME_V4{});
  HS_LIDAR_TAIL_ME_V4 tail{};
  tail.m_u8ReturnMode = 0x37;
  tail.m_u16MotorSpeed = 12000;  // JT128 reports RPM * 10
  // UTC's alternate encoding is big-endian Unix seconds: 1700000000.
  tail.m_u8UTC[2] = 0x65; tail.m_u8UTC[3] = 0x53;
  tail.m_u8UTC[4] = 0xf1; tail.m_u8UTC[5] = 0x00;
  tail.m_u32Timestamp = 125000;
  append(tail);
  HS_LIDAR_TAIL_IMU_ME_V4 imu{};
  imu.m_u16IMUAccelUnit = 1000;
  imu.m_u16IMUAngVelUnit = 100;
  imu.m_i16IMUZAccel = 1000;
  imu.m_i16IMUZAngVel = 10000;
  append(imu);
  append(HS_LIDAR_TAIL_CRC_ME_V4{});
  EXPECT_EQ(bytes.size(), header.packetSize());
  return UdpPacket(bytes.data(), bytes.size(), receive_time);
}

std::string flatCalibration() {
  std::ostringstream csv;
  csv << "Laser id,Elevation,Azimuth\n";
  for (int i = 1; i <= 128; ++i) csv << i << ",0,0\n";
  return csv.str();
}

class Jt128Timestamp : public ::testing::TestWithParam<int> {};
}

TEST_P(Jt128Timestamp, DecodeThroughRosPreservesSelectedClockAndSIUnits) {
  // The SDK parser owns large trigonometric tables; allocate it as the SDK does.
  auto parser = std::make_unique<Udp1_4Parser<LidarPointXYZIRT>>(STR_OTHER);
  const auto calibration = flatCalibration();
  ASSERT_EQ(parser->LoadCorrectionString(calibration.data(), calibration.size()), 0);
  LidarDecodedFrame<LidarPointXYZIRT> frame(1, 256);
  frame.fParam.use_timestamp_type = GetParam();
  const double expected = GetParam() == 0 ? 1700000000.125 : 1700000001.25;
  ASSERT_EQ(parser->DecodePacket(frame, jt128Packet(1700000001250000ULL)), 0);
  ASSERT_EQ(parser->ComputeXYZI(frame, 0), 0);
  ASSERT_EQ(frame.valid_points[0], 256);
  frame.points_num = frame.valid_points[0];
  EXPECT_DOUBLE_EQ(frame.frame_start_timestamp, expected);
  EXPECT_DOUBLE_EQ(frame.frame_end_timestamp, expected);
  for (size_t i = 0; i < frame.points_num; ++i) {
    const auto & p = frame.points[i];
    EXPECT_DOUBLE_EQ(p.timestamp, expected);
    EXPECT_EQ(p.ring, i % 128);
    EXPECT_FLOAT_EQ(p.intensity, i % 128);
    EXPECT_NEAR(p.x, i < 128 ? 0 : 2, 1e-5);
    EXPECT_NEAR(p.y, i < 128 ? 2 : 0, 1e-5);
    EXPECT_NEAR(p.z, 0, 1e-5);
  }
  ConversionDriver driver;
  const auto imu = driver.ToRosMsg(frame.imu_config);
  EXPECT_NEAR(imu.linear_acceleration.z, 9.80665, 1e-12);
  EXPECT_NEAR(imu.angular_velocity.z, M_PI / 18, 1e-12);
  driver.compact();
  const auto cloud = driver.ToRosMsg(frame, "hesai_lidar");
  EXPECT_EQ(cloud.header.stamp, imu.header.stamp);
  EXPECT_EQ(cloud.header.stamp.sec, int32_t(expected));
  EXPECT_EQ(cloud.header.stamp.nanosec, GetParam() == 0 ? 125000000U : 250000000U);
  EXPECT_DOUBLE_EQ(field<double>(cloud, 0, 18), expected);

#ifdef USE_CUDA
  int devices = 0;
  if (cudaGetDeviceCount(&devices) != cudaSuccess || devices == 0)
    GTEST_SKIP() << "CPU assertions passed; GPU parity requires a CUDA device";
  const std::vector<LidarPointXYZIRT> cpu(frame.points, frame.points + frame.points_num);
  Udp1_4ParserGpu<LidarPointXYZIRT> gpu(STR_OTHER, 1, 256);
  bool correction_loaded = true, firetime_loaded = false;
  uint32_t correction_sequence = 0, firetime_sequence = 0;
  gpu.setCorrectionLoadFlag(&correction_loaded);
  gpu.setFiretimeLoadFlag(&firetime_loaded);
  gpu.setCorrectionLoadSequenceNum(&correction_sequence);
  gpu.setFiretimeLoadSequenceNum(&firetime_sequence);
  gpu.LoadCorrectionStruct(parser->getStruct(CORRECTION_STRUCT));
  ASSERT_EQ(gpu.ComputeXYZI(frame), 0);
  ASSERT_EQ(frame.valid_points[0], cpu.size());
  for (size_t i = 0; i < cpu.size(); ++i) {
    EXPECT_NEAR(frame.points[i].x, cpu[i].x, 1e-5);
    EXPECT_NEAR(frame.points[i].y, cpu[i].y, 1e-5);
    EXPECT_NEAR(frame.points[i].z, cpu[i].z, 1e-5);
    EXPECT_DOUBLE_EQ(frame.points[i].timestamp, cpu[i].timestamp);
    EXPECT_EQ(frame.points[i].ring, cpu[i].ring);
    EXPECT_EQ(frame.points[i].intensity, cpu[i].intensity);
  }
#endif
}

INSTANTIATE_TEST_SUITE_P(SensorAndReceiveTime, Jt128Timestamp, ::testing::Values(0, 1));
