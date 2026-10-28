#include <gtest/gtest.h>
#include <chrono>
#include "cloud_output_filter.hpp"

namespace {
struct Point { float x, y, z, intensity; uint16_t ring; double timestamp; };
template<class T> T read(const sensor_msgs::msg::PointCloud2 &cloud, size_t index, size_t offset) {
  T v; std::memcpy(&v, cloud.data.data() + index*cloud.point_step + offset, sizeof(v)); return v;
}
hesai_ros::CloudOutputFilter config() {
  hesai_ros::CloudOutputFilter f;
  f.enabled = true; f.stride = 2; f.crop_enabled = true;
  f.crop_min = {{-0.5,-0.5,-0.5}}; f.crop_max = {{0.5,0.5,0.5}};
  f.min_range = .5; f.max_range = 60;
  return f;
}
}
TEST(CloudOutput, CropThenDecimatePreservesFieldsAndScanEnd) {
  // Surviving crop indices: 1,2,4,5,6. Decimation selects 1,4,6;
  // range then removes 6. Last timestamp must survive as timing-only NaN.
  Point p[] = {{0,0,0,1,0,10}, {1,0,0,2,7,10.01}, {2,0,0,3,8,10.02},
    {.5,0,0,4,9,10.03}, {3,0,0,5,10,10.04}, {4,0,0,6,11,10.05}, {61,0,0,7,12,10.06}};
  hesai_ros::CloudOutputStats stats;
  auto f = config(); f.validate();
  const auto c = hesai_ros::makeOutputCloud(p, 7, 10, "lidar", f, stats);
  ASSERT_EQ(c.width, 3U); EXPECT_EQ(c.point_step, 28U);
  EXPECT_EQ(stats.crop_removed, 2U); EXPECT_EQ(stats.range_removed, 1U);
  EXPECT_EQ(c.header.frame_id, "lidar"); EXPECT_EQ(c.header.stamp.sec, 10);
  EXPECT_EQ(read<float>(c,0,0), 1); EXPECT_EQ(read<float>(c,1,0), 3);
  EXPECT_EQ(read<float>(c,1,12), 5); EXPECT_EQ(read<uint16_t>(c,1,16), 10);
  EXPECT_DOUBLE_EQ(read<double>(c,1,18), 10.04);
  EXPECT_EQ(read<uint16_t>(c,2,26), 2);
  EXPECT_TRUE(std::isnan(read<float>(c,2,0)));
  EXPECT_DOUBLE_EQ(read<double>(c,2,18), 10.06);
}
TEST(CloudOutput, StaticMountingAndInclusiveBox) {
  auto f = config(); f.rotation = {{0,1,0,-1,0,0,0,0,1}};
  f.translation = {{0,0,1}}; f.stride = 1;
  Point p[] = {{0,0,-1,1,0,10}, {1,0,-1,1,0,10.01}, {.5,0,-1.5,1,0,10.02}};
  hesai_ros::CloudOutputStats stats;
  auto c = hesai_ros::makeOutputCloud(p,3,10,"lidar",f,stats);
  ASSERT_EQ(stats.selected_points,1U); EXPECT_EQ(stats.crop_removed,2U);
  EXPECT_FLOAT_EQ(read<float>(c,0,0),1); EXPECT_FLOAT_EQ(read<float>(c,0,8),-1);
  EXPECT_TRUE(stats.timing_point);
}
TEST(CloudOutput, RawLayoutAndTimestampUnchanged) {
  Point p[] = {{1,2,3,4,5,1700000000.1234567}};
  hesai_ros::CloudOutputStats stats; hesai_ros::CloudOutputFilter f;
  auto c = hesai_ros::makeOutputCloud(p,1,p[0].timestamp,"lidar",f,stats);
  EXPECT_EQ(c.width,1U); EXPECT_EQ(c.fields.size(),6U); EXPECT_EQ(c.point_step,26U);
  EXPECT_DOUBLE_EQ(read<double>(c,0,18),p[0].timestamp);
  EXPECT_EQ(c.header.stamp.nanosec, std::llround((p[0].timestamp-1700000000)*1e9));
}
TEST(CloudOutput, BadTimestampCannotBeHiddenByDecimationOrCrop) {
  Point p[] = {{1,0,0,1,0,10}, {0,0,0,1,0,11}};
  auto f = config(); hesai_ros::CloudOutputStats stats;
  EXPECT_THROW(hesai_ros::makeOutputCloud(p,2,10,"lidar",f,stats),std::invalid_argument);
  p[1].timestamp = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(hesai_ros::makeOutputCloud(p,2,10,"lidar",f,stats),std::invalid_argument);
}
TEST(CloudOutput, InvalidCoordinatesDoNotShiftPostCropDecimation) {
  const float nan = std::numeric_limits<float>::quiet_NaN();
  Point p[] = {{nan,0,0,1,0,10}, {1,0,0,1,0,10.01}, {2,0,0,1,0,10.02}, {3,0,0,1,0,10.03}};
  auto f = config(); hesai_ros::CloudOutputStats stats;
  auto c = hesai_ros::makeOutputCloud(p,4,10,"lidar",f,stats);
  ASSERT_EQ(c.width,2U); EXPECT_EQ(read<float>(c,0,0),1); EXPECT_EQ(read<float>(c,1,0),3);
  EXPECT_FALSE(stats.timing_point);
}
TEST(CloudOutput, InvalidConfigurationAndEmptyCloud) {
  auto f = config(); f.stride = 0; EXPECT_THROW(f.validate(),std::invalid_argument);
  f = config(); f.rotation[0] = 2; EXPECT_THROW(f.validate(),std::invalid_argument);
  f = config(); f.max_range = -1; EXPECT_THROW(f.validate(),std::invalid_argument);
  f = config(); Point p[] = {{0,0,0,1,0,10}}; hesai_ros::CloudOutputStats stats;
  EXPECT_EQ(hesai_ros::makeOutputCloud(p,1,10,"lidar",f,stats).width,0U);
}
TEST(CloudOutput, RepresentativeFullScanCostAndSize) {
  std::vector<Point> points(230400);
  for (size_t i=0; i<points.size(); ++i) points[i] = {2,0,1,1,uint16_t(i%128),10+.099*i/points.size()};
  auto f = config(); f.stride = 6;
  hesai_ros::CloudOutputStats stats;
  std::vector<double> costs;
  for (int n=0;n<100;++n) {
    auto start = std::chrono::steady_clock::now();
    auto c = hesai_ros::makeOutputCloud(points.data(),points.size(),10,"lidar",f,stats);
    costs.push_back(std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count());
    ASSERT_EQ(c.width,38401U); ASSERT_EQ(c.data.size(),1075228U);
  }
  std::sort(costs.begin(),costs.end());
  std::cout << "230400 input -> 38400 features + timing: conversion p50=" << costs[50]
            << " ms p99=" << costs[98] << " ms max=" << costs.back() << " ms\n";
}
