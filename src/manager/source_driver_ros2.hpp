/************************************************************************************************
  Copyright(C)2023 Hesai Technology Co., Ltd.
  All code in this repository is released under the terms of the following [Modified BSD License.]
  Modified BSD License:
  Redistribution and use in source and binary forms,with or without modification,are permitted 
  provided that the following conditions are met:
  *Redistributions of source code must retain the above copyright notice,this list of conditions 
   and the following disclaimer.
  *Redistributions in binary form must reproduce the above copyright notice,this list of conditions and 
   the following disclaimer in the documentation and/or other materials provided with the distribution.
  *Neither the names of the University of Texas at Austin,nor Austin Robot Technology,nor the names of 
   other contributors maybe used to endorse or promote products derived from this software without 
   specific prior written permission.
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGH THOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED 
  WARRANTIES,INCLUDING,BUT NOT LIMITED TO,THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A 
  PARTICULAR PURPOSE ARE DISCLAIMED.IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR 
  ANY DIRECT,INDIRECT,INCIDENTAL,SPECIAL,EXEMPLARY,OR CONSEQUENTIAL DAMAGES(INCLUDING,BUT NOT LIMITED TO,
  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;LOSS OF USE,DATA,OR PROFITS;OR BUSINESS INTERRUPTION)HOWEVER 
  CAUSED AND ON ANY THEORY OF LIABILITY,WHETHER IN CONTRACT,STRICT LIABILITY,OR TORT(INCLUDING NEGLIGENCE 
  OR OTHERWISE)ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,EVEN IF ADVISED OF THE POSSIBILITY OF 
  SUCHDAMAGE.
************************************************************************************************/

/*
 * File: source_driver_ros2.hpp
 * Author: Zhang Yu <zhangyu@hesaitech.com>
 * Description: Source Driver for ROS2
 * Created on June 12, 2023, 10:46 AM
 */

#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sstream>
#include <hesai_ros_driver/msg/udp_frame.hpp>
#include <hesai_ros_driver/msg/udp_packet.hpp>
#include <hesai_ros_driver/msg/ptp.hpp>
#include <hesai_ros_driver/msg/firetime.hpp>
#include <hesai_ros_driver/msg/loss_packet.hpp>

#include <fstream>
#include <memory>
#include <chrono>
#include <string>
#include <functional>
#include <boost/thread.hpp>
#include "source_drive_common.hpp"
#include "cloud_output_filter.hpp"
#include <diagnostic_msgs/msg/diagnostic_array.hpp>

class SourceDriver
{
public:
  typedef std::shared_ptr<SourceDriver> Ptr;
  // Initialize some necessary configuration parameters, create ROS nodes, and register callback functions
  virtual void Init(const YAML::Node& config);
  // Start working
  virtual void Start();
  // Stop working
  virtual void Stop();
  virtual ~SourceDriver();
  SourceDriver(SourceType src_type) {};
  void SpinRos2(){rclcpp::spin(this->node_ptr_);}
  std::shared_ptr<rclcpp::Node> node_ptr_;
  std::shared_ptr<HesaiLidarSdk<LidarPointXYZIRT>> driver_ptr_;
protected:
  // Save Correction file subscribed by "ros_recv_correction_topic"
  void ReceiveCorrection(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);
  // Save packets subscribed by 'ros_recv_packet_topic'
  void ReceivePacket(const hesai_ros_driver::msg::UdpFrame::SharedPtr msg);
  // Used to publish point clouds through 'ros_send_point_cloud_topic'
  void SendPointCloud(const LidarDecodedFrame<LidarPointXYZIRT>& msg);
  // Used to publish the original packet through 'ros_send_packet_topic'
  void SendPacket(const UdpFrame_t& ros_msg, double timestamp);

  // Used to publish the Correction file through 'ros_send_correction_topic'
  void SendCorrection(const u8Array_t& msg);
  // Used to publish the Packet loss condition
  void SendPacketLoss(const uint32_t& total_packet_count, const uint32_t& total_packet_loss_count);
  // Used to publish the Packet loss condition
  void SendPTP(const uint8_t& ptp_lock_offset, const u8Array_t& ptp_status);
  // Used to publish the imu packet
  void SendImuConfig(const LidarImuData& msg);
  void InitImuPublishers(const std::string &topic);
  // Publish raw data packets, sending them one by one
  void SendPacketOneByOne(const UdpPacket& ros_msg, double timestamp);

  // Convert ptp lock offset, status into ROS message
  hesai_ros_driver::msg::Ptp ToRosMsg(const uint8_t& ptp_lock_offset, const u8Array_t& ptp_status);
  // Convert packet loss condition into ROS message
  hesai_ros_driver::msg::LossPacket ToRosMsg(const uint32_t& total_packet_count, const uint32_t& total_packet_loss_count);
  // Convert correction string into ROS messages
  std_msgs::msg::UInt8MultiArray ToRosMsg(const u8Array_t& correction_string);
  // Convert double[512] to float64[512]
  hesai_ros_driver::msg::Firetime ToRosMsg(const double *firetime_correction_);
  // Convert point clouds into ROS messages
  sensor_msgs::msg::PointCloud2 ToRosMsg(const LidarDecodedFrame<LidarPointXYZIRT>& frame, const std::string& frame_id);
  // Convert packets into ROS messages
  hesai_ros_driver::msg::UdpFrame ToRosMsg(const UdpFrame_t& ros_msg, double timestamp);
  hesai_ros_driver::msg::UdpPacket ToRosMsg(const UdpPacket& ros_msg, double timestamp);
  // Convert imu, imu into ROS message
  sensor_msgs::msg::Imu ToRosMsg(const LidarImuData& firetime_correction_);
  // The bundled JT128 parser still reports g and degrees/s. Keep ROS SI units.
  double From_g_To_ms2(double g);
  // Convert Angular Velocity from degree/s to radian/s
  double From_degs_To_rads(double degree);
  std::string frame_id_;

  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr crt_sub_;
  rclcpp::Subscription<hesai_ros_driver::msg::UdpFrame>::SharedPtr pkt_sub_;
  rclcpp::Publisher<hesai_ros_driver::msg::UdpFrame>::SharedPtr pkt_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Publisher<hesai_ros_driver::msg::Firetime>::SharedPtr firetime_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr crt_pub_;
  rclcpp::Publisher<hesai_ros_driver::msg::LossPacket>::SharedPtr loss_pub_;
  rclcpp::Publisher<hesai_ros_driver::msg::Ptp>::SharedPtr ptp_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<hesai_ros_driver::msg::UdpPacket>::SharedPtr every_pkt_pub_;

  hesai_ros::CloudOutputFilter output_filter_;
  hesai_ros::CloudOutputStats output_stats_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr cloud_diag_pub_;
  std::chrono::steady_clock::time_point last_cloud_receipt_{};
  int requested_rpm_ = -1, requested_return_mode_ = -1;
  uint64_t cloud_drop_count_ = 0;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr imu_diag_pub_;
  std::chrono::steady_clock::time_point last_imu_callback_{}, last_imu_diagnostic_{};
  double last_imu_stamp_ = 0.0, max_imu_stamp_gap_ = 0.0;
  double max_imu_callback_gap_ = 0.0, max_imu_publish_ms_ = 0.0;
  uint64_t imu_published_ = 0, imu_stamp_gaps_ = 0, imu_non_increasing_ = 0;

  //spin thread while Receive data from ROS topic
  boost::thread* subscription_spin_thread_;
};
inline void SourceDriver::Init(const YAML::Node& config)
{
  DriverParam driver_param;
  DriveYamlParam yaml_param;
  yaml_param.GetDriveYamlParam(config, driver_param);
  frame_id_ = driver_param.input_param.frame_id;

  node_ptr_.reset(new rclcpp::Node("hesai_ros_driver_node"));
  output_filter_.enabled = node_ptr_->declare_parameter("output_filter.enabled", false);
  const int stride = node_ptr_->declare_parameter("output_filter.stride", 1);
  if (stride < 1 || stride > 65535) throw std::invalid_argument("invalid output_filter.stride");
  output_filter_.stride = static_cast<uint16_t>(stride);
  output_filter_.crop_enabled = node_ptr_->declare_parameter("output_filter.crop_enabled", false);
  auto read_array = [this](const std::string &name, auto &destination) {
    const std::vector<double> defaults(destination.begin(), destination.end());
    const auto values = node_ptr_->declare_parameter<std::vector<double>>(name, defaults);
    if (values.size() != destination.size()) throw std::invalid_argument(name + ": wrong size");
    std::copy(values.begin(), values.end(), destination.begin());
  };
  read_array("output_filter.crop_min", output_filter_.crop_min);
  read_array("output_filter.crop_max", output_filter_.crop_max);
  read_array("output_filter.rotation", output_filter_.rotation);
  read_array("output_filter.translation", output_filter_.translation);
  output_filter_.min_range = node_ptr_->declare_parameter("output_filter.min_range", 0.0);
  output_filter_.max_range = node_ptr_->declare_parameter("output_filter.max_range", 1000.0);
  output_filter_.validate();
  requested_rpm_ = node_ptr_->declare_parameter("requested_spin_speed", -1);
  requested_return_mode_ = node_ptr_->declare_parameter("requested_return_mode", -1);
  if (!((requested_rpm_ == -1 && requested_return_mode_ == -1) ||
        (requested_rpm_ == 1200 && requested_return_mode_ == 1) ||
        (requested_rpm_ == 600 && requested_return_mode_ == 2)))
    throw std::invalid_argument("supported modes: unchanged, 1200/strongest, 600/dual");
  if (requested_rpm_ != -1 && driver_param.input_param.source_type != DATA_FROM_LIDAR)
    throw std::invalid_argument("sensor mode changes require a live LiDAR source");
  cloud_diag_pub_ = node_ptr_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "pointcloud_diagnostics", 10);
  if (driver_param.input_param.send_point_cloud_ros) {
    pub_ = node_ptr_->create_publisher<sensor_msgs::msg::PointCloud2>(driver_param.input_param.ros_send_point_topic, 10);
  }
  if (driver_param.input_param.send_imu_ros) {
    InitImuPublishers(driver_param.input_param.ros_send_imu_topic);
  }

  if (driver_param.input_param.ros_send_packet_loss_topic != NULL_TOPIC) {
    loss_pub_ = node_ptr_->create_publisher<hesai_ros_driver::msg::LossPacket>(driver_param.input_param.ros_send_packet_loss_topic, 10);
  }

  rclcpp::QoS qos_settings(10);
  qos_settings.transient_local();
  if (driver_param.input_param.source_type == DATA_FROM_LIDAR) {
    if (driver_param.input_param.ros_send_ptp_topic != NULL_TOPIC) {
      ptp_pub_ = node_ptr_->create_publisher<hesai_ros_driver::msg::Ptp>(driver_param.input_param.ros_send_ptp_topic, qos_settings);
    }

    if (driver_param.input_param.ros_send_correction_topic != NULL_TOPIC) {
      crt_pub_ = node_ptr_->create_publisher<std_msgs::msg::UInt8MultiArray>(driver_param.input_param.ros_send_correction_topic, qos_settings);
    }
  }

  if (driver_param.input_param.send_packet_ros) {
    pkt_pub_ = node_ptr_->create_publisher<hesai_ros_driver::msg::UdpFrame>(driver_param.input_param.ros_send_packet_topic, 10);
  }
  if (driver_param.input_param.ros_send_every_packet_topic != NULL_TOPIC) {
    every_pkt_pub_ = node_ptr_->create_publisher<hesai_ros_driver::msg::UdpPacket>(driver_param.input_param.ros_send_every_packet_topic, 10);
  }

  if (driver_param.input_param.source_type == DATA_FROM_ROS_PACKET) {
    pkt_sub_ = node_ptr_->create_subscription<hesai_ros_driver::msg::UdpFrame>(driver_param.input_param.ros_recv_packet_topic, 10, 
                              std::bind(&SourceDriver::ReceivePacket, this, std::placeholders::_1));
    if (driver_param.input_param.ros_recv_correction_topic != NULL_TOPIC) {    
      crt_sub_ = node_ptr_->create_subscription<std_msgs::msg::UInt8MultiArray>(driver_param.input_param.ros_recv_correction_topic, 10, 
                              std::bind(&SourceDriver::ReceiveCorrection, this, std::placeholders::_1));
    }
    driver_param.decoder_param.enable_udp_thread = false;
    subscription_spin_thread_ = new boost::thread(boost::bind(&SourceDriver::SpinRos2,this));
  }
  driver_ptr_.reset(new HesaiLidarSdk<LidarPointXYZIRT>());
  driver_param.decoder_param.enable_parser_thread = true;
  if (driver_param.input_param.send_point_cloud_ros) {
    driver_ptr_->RegRecvCallback([this](const hesai::lidar::LidarDecodedFrame<hesai::lidar::LidarPointXYZIRT>& frame) {  
      this->SendPointCloud(frame);  
    });  
  }
  if (driver_param.input_param.send_imu_ros) {
    driver_ptr_->RegRecvCallback(std::bind(&SourceDriver::SendImuConfig, this, std::placeholders::_1));
  }
  if (driver_param.input_param.send_packet_ros) {
    driver_ptr_->RegRecvCallback(std::bind(&SourceDriver::SendPacket, this, std::placeholders::_1, std::placeholders::_2)) ;
  }
  if (driver_param.input_param.ros_send_every_packet_topic != NULL_TOPIC) {
    driver_ptr_->RegRecvCallback(std::bind(&SourceDriver::SendPacketOneByOne, this, std::placeholders::_1, std::placeholders::_2));
  }
  if (driver_param.input_param.ros_send_packet_loss_topic != NULL_TOPIC) {
    driver_ptr_->RegRecvCallback(std::bind(&SourceDriver::SendPacketLoss, this, std::placeholders::_1, std::placeholders::_2));
  }
  if (driver_param.input_param.source_type == DATA_FROM_LIDAR) {
    if (driver_param.input_param.ros_send_correction_topic != NULL_TOPIC) {
      driver_ptr_->RegRecvCallback(std::bind(&SourceDriver::SendCorrection, this, std::placeholders::_1));
    }
    if (driver_param.input_param.ros_send_ptp_topic != NULL_TOPIC) {
      driver_ptr_->RegRecvCallback(std::bind(&SourceDriver::SendPTP, this, std::placeholders::_1, std::placeholders::_2));
    }
  } 
  if (!driver_ptr_->Init(driver_param))
  {
    std::cout << "Driver Initialize Error...." << std::endl;
    exit(-1);
  }
}

inline void SourceDriver::Start()
{
  if (requested_rpm_ != -1) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
    while (!driver_ptr_->lidar_ptr_->GetInitFinish(AllInitFinish)) {
      if (!rclcpp::ok() || std::chrono::steady_clock::now() > deadline)
        throw std::runtime_error("timed out waiting to configure JT128 mode");
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    auto lidar = driver_ptr_->lidar_ptr_;
    if (lidar->GetLidarType() != "JT128" || !lidar->ptc_client_)
      throw std::runtime_error("explicit sensor mode profiles require JT128 with PTC");
    // Only these two mode commands are sent. Network/calibration settings are untouched.
    if (!lidar->ptc_client_->SetReturnMode(static_cast<uint8_t>(requested_return_mode_)) ||
        !lidar->ptc_client_->SetSpinSpeed(static_cast<uint32_t>(requested_rpm_)))
      throw std::runtime_error("JT128 rejected mode change; inspect sensor mode before restarting");
    RCLCPP_INFO(node_ptr_->get_logger(), "Requested JT128 %d RPM, return mode %d",
      requested_rpm_, requested_return_mode_);
  }
  driver_ptr_->Start();
}

inline SourceDriver::~SourceDriver()
{
  Stop();
}

inline void SourceDriver::Stop()
{
  if (driver_ptr_) driver_ptr_->Stop();
}

inline void SourceDriver::SendPacket(const UdpFrame_t& msg, double timestamp)
{
  pkt_pub_->publish(ToRosMsg(msg, timestamp));
}

inline void SourceDriver::SendPointCloud(const LidarDecodedFrame<LidarPointXYZIRT>& msg)
{
  using Clock = std::chrono::steady_clock;
  const auto start = Clock::now();
  const double interval_ms = last_cloud_receipt_ == Clock::time_point{} ? 0.0 :
    std::chrono::duration<double, std::milli>(start - last_cloud_receipt_).count();
  last_cloud_receipt_ = start;
  diagnostic_msgs::msg::DiagnosticArray diag;
  diag.header.stamp = node_ptr_->now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "hesai/pointcloud_output";
  status.hardware_id = frame_id_;
  auto add = [&status](const std::string &key, double value) {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = key; kv.value = std::to_string(value); status.values.push_back(kv);
  };
  add("frame_index", msg.frame_index);
  add("scan_start", msg.frame_start_timestamp);
  add("sdk_interval_ms", interval_ms);
  add("return_mode", msg.return_mode);
  const double scan_span = msg.frame_end_timestamp - msg.frame_start_timestamp;
  add("scan_span_ms", scan_span * 1000);
  try {
    if (requested_rpm_ != -1 &&
        (scan_span < (requested_rpm_ == 1200 ? 0.025 : 0.075) ||
         scan_span > (requested_rpm_ == 1200 ? 0.075 : 0.125) ||
         msg.return_mode != (requested_return_mode_ == 1 ? 0x37 : 0x39)))
      throw std::runtime_error("waiting for requested RPM/return mode in sensor packets");
    auto cloud = ToRosMsg(msg, frame_id_);
    const auto converted = Clock::now();
    add("scan_end", output_stats_.scan_end);
    add("scan_end_age_ms", (node_ptr_->now().seconds() - output_stats_.scan_end) * 1000);
    add("input_points", output_stats_.input_points);
    add("usable_points", output_stats_.selected_points);
    add("crop_removed", output_stats_.crop_removed);
    add("invalid_points", output_stats_.invalid_points);
    add("range_removed", output_stats_.range_removed);
    add("timing_point", output_stats_.timing_point);
    add("output_bytes", cloud.data.size());
    add("stride", output_filter_.enabled ? output_filter_.stride : 1);
    add("conversion_ms", std::chrono::duration<double, std::milli>(converted - start).count());
    if (cloud.width == 0) throw std::runtime_error("no usable cloud points");
    pub_->publish(cloud);
    add("publish_ms", std::chrono::duration<double, std::milli>(Clock::now() - converted).count());
    status.message = "published";
  } catch (const std::exception &error) {
    ++cloud_drop_count_;
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = error.what();
    RCLCPP_WARN_THROTTLE(node_ptr_->get_logger(), *node_ptr_->get_clock(), 2000,
      "Dropping point cloud: %s", error.what());
  }
  add("dropped_frames", cloud_drop_count_);
  add("callback_ms", std::chrono::duration<double, std::milli>(Clock::now() - start).count());
  diag.status.push_back(std::move(status));
  cloud_diag_pub_->publish(diag);
}

inline void SourceDriver::SendCorrection(const u8Array_t& msg)
{
  crt_pub_->publish(ToRosMsg(msg));
}

inline void SourceDriver::SendPacketLoss(const uint32_t& total_packet_count, const uint32_t& total_packet_loss_count)
{
  loss_pub_->publish(ToRosMsg(total_packet_count, total_packet_loss_count));
}

inline void SourceDriver::SendPTP(const uint8_t& ptp_lock_offset, const u8Array_t& ptp_status)
{
  ptp_pub_->publish(ToRosMsg(ptp_lock_offset, ptp_status));
}

inline void SourceDriver::SendImuConfig(const LidarImuData& msg)
{
  using Clock = std::chrono::steady_clock;
  const auto start = Clock::now();
  const double stamp_gap = imu_published_ ? msg.timestamp - last_imu_stamp_ : 0.0;
  const double callback_gap = imu_published_ ?
    std::chrono::duration<double>(start - last_imu_callback_).count() : 0.0;
  max_imu_stamp_gap_ = std::max(max_imu_stamp_gap_, stamp_gap);
  max_imu_callback_gap_ = std::max(max_imu_callback_gap_, callback_gap);
  if (stamp_gap > 0.1) ++imu_stamp_gaps_;
  if (imu_published_ && stamp_gap <= 0.0) ++imu_non_increasing_;
  last_imu_stamp_ = msg.timestamp;
  last_imu_callback_ = start;
  imu_pub_->publish(ToRosMsg(msg));
  const double publish_ms = std::chrono::duration<double, std::milli>(Clock::now() - start).count();
  max_imu_publish_ms_ = std::max(max_imu_publish_ms_, publish_ms);
  ++imu_published_;
  // SDK callbacks are serialized. Report from the publishing side, allowing
  // bags to distinguish an SDK/source gap from a downstream ROS reader gap.
  if (imu_published_ == 1 || stamp_gap > 0.1 || callback_gap > 0.1 ||
      start - last_imu_diagnostic_ >= std::chrono::seconds(1)) {
    last_imu_diagnostic_ = start;
    diagnostic_msgs::msg::DiagnosticArray diag;
    diag.header.stamp = node_ptr_->now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "hesai/imu_delivery";
    status.hardware_id = frame_id_;
    status.level = imu_stamp_gaps_ ? diagnostic_msgs::msg::DiagnosticStatus::WARN :
                                   diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = imu_stamp_gaps_ ? "published_with_source_stamp_gaps" : "published";
    auto add = [&status](const char *key, double value) {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = key; kv.value = std::to_string(value); status.values.push_back(kv);
    };
    add("published_samples", imu_published_);
    add("stamp", msg.timestamp);
    add("stamp_gap_seconds", stamp_gap);
    add("max_stamp_gap_seconds", max_imu_stamp_gap_);
    add("stamp_gaps_over_100ms", imu_stamp_gaps_);
    add("non_increasing_stamps", imu_non_increasing_);
    add("callback_gap_seconds", callback_gap);
    add("max_callback_gap_seconds", max_imu_callback_gap_);
    add("source_age_ms", (node_ptr_->now().seconds() - msg.timestamp) * 1000);
    add("publish_ms", publish_ms);
    add("max_publish_ms", max_imu_publish_ms_);
    add("qos_depth", imu_pub_->get_actual_qos().depth());
    diag.status.push_back(std::move(status));
    imu_diag_pub_->publish(diag);
  }
}

inline void SourceDriver::InitImuPublishers(const std::string &topic)
{
  // 20 samples only covered about 50 ms at 400 Hz. Retain bursts caused by
  // startup scheduling / DDS discovery without filling gaps or changing time.
  const int depth = node_ptr_->declare_parameter("imu_qos_depth", 2000);
  if (depth < 1 || depth > 100000) throw std::invalid_argument("invalid imu_qos_depth");
  imu_pub_ = node_ptr_->create_publisher<sensor_msgs::msg::Imu>(
    topic, rclcpp::QoS(depth).reliable().durability_volatile());
  imu_diag_pub_ = node_ptr_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "imu_diagnostics", 10);
}

inline void SourceDriver::SendPacketOneByOne(const UdpPacket& msg, double timestamp)
{
  every_pkt_pub_->publish(ToRosMsg(msg, timestamp));
}
inline sensor_msgs::msg::PointCloud2 SourceDriver::ToRosMsg(const LidarDecodedFrame<LidarPointXYZIRT>& frame, const std::string& frame_id)
{
  const bool multi = frame.fParam.IsMultiFrameFrequency() != 0;
  return hesai_ros::makeOutputCloud(
    multi ? frame.multi_points : frame.points,
    multi ? frame.multi_points_num : frame.points_num,
    multi ? frame.multi_frame_start_timestamp : frame.frame_start_timestamp,
    frame_id, output_filter_, output_stats_);
}

inline hesai_ros_driver::msg::UdpPacket SourceDriver::ToRosMsg(const UdpPacket& ros_msg, double timestamp) {
  hesai_ros_driver::msg::UdpPacket rawpacket;
  rawpacket.size = ros_msg.packet_len;
  rawpacket.data.resize(ros_msg.packet_len);
  memcpy(&rawpacket.data[0], &ros_msg.buffer[0], ros_msg.packet_len);
  rawpacket.stamp.sec = (uint32_t)floor(timestamp);
  rawpacket.stamp.nanosec = (uint32_t)round((timestamp - rawpacket.stamp.sec) * 1e9);
  return rawpacket;
}
inline hesai_ros_driver::msg::UdpFrame SourceDriver::ToRosMsg(const UdpFrame_t& ros_msg, double timestamp) {
  hesai_ros_driver::msg::UdpFrame rs_msg;
  for (size_t i = 0 ; i < ros_msg.size(); i++) {
    hesai_ros_driver::msg::UdpPacket rawpacket;
    rawpacket.size = ros_msg[i].packet_len;
    rawpacket.data.resize(ros_msg[i].packet_len);
    memcpy(&rawpacket.data[0], &ros_msg[i].buffer[0], ros_msg[i].packet_len);
    rs_msg.packets.push_back(rawpacket);
  }
  auto sec = (uint64_t)floor(timestamp);
  if (sec <= std::numeric_limits<int32_t>::max()) {
    rs_msg.header.stamp.sec = (uint32_t)floor(timestamp);
    rs_msg.header.stamp.nanosec = (uint32_t)round((timestamp - rs_msg.header.stamp.sec) * 1e9);
  } else {
    RCLCPP_ERROR(node_ptr_->get_logger(), "does not support timestamps greater than 19 January 2038 03:14:07 (now %lf)\n", timestamp);
  }
  rs_msg.header.frame_id = frame_id_;
  return rs_msg;
}

inline std_msgs::msg::UInt8MultiArray SourceDriver::ToRosMsg(const u8Array_t& correction_string) {
  auto msg = std::make_shared<std_msgs::msg::UInt8MultiArray>();
  msg->data.resize(correction_string.size());
  std::copy(correction_string.begin(), correction_string.end(), msg->data.begin());
  return *msg;
}

inline hesai_ros_driver::msg::LossPacket SourceDriver::ToRosMsg(const uint32_t& total_packet_count, const uint32_t& total_packet_loss_count)
{
  hesai_ros_driver::msg::LossPacket msg;
  msg.total_packet_count = total_packet_count;
  msg.total_packet_loss_count = total_packet_loss_count;  
  return msg;
}

inline hesai_ros_driver::msg::Ptp SourceDriver::ToRosMsg(const uint8_t& ptp_lock_offset, const u8Array_t& ptp_status)
{
  hesai_ros_driver::msg::Ptp msg;
  msg.ptp_lock_offset = ptp_lock_offset;
  msg.ptp_status_size = ptp_status.size();
  msg.ptp_status.resize(ptp_status.size());
  std::copy(ptp_status.begin(), ptp_status.end(), msg.ptp_status.begin());
  return msg;
}

inline hesai_ros_driver::msg::Firetime SourceDriver::ToRosMsg(const double *firetime_correction_)
{
  hesai_ros_driver::msg::Firetime msg;
  std::copy(firetime_correction_, firetime_correction_ + 512, msg.data.begin());
  return msg;
}

inline sensor_msgs::msg::Imu SourceDriver::ToRosMsg(const LidarImuData &imu_config_)
{
  sensor_msgs::msg::Imu ros_msg;
  auto sec = (uint64_t)floor(imu_config_.timestamp);
  if (sec <= std::numeric_limits<int32_t>::max()) {
    ros_msg.header.stamp.sec = (uint32_t)floor(imu_config_.timestamp);
    ros_msg.header.stamp.nanosec = (uint32_t)round((imu_config_.timestamp - ros_msg.header.stamp.sec) * 1e9);
  } else {
    RCLCPP_ERROR(node_ptr_->get_logger(), "does not support timestamps greater than 19 January 2038 03:14:07 (now %lf)\n", imu_config_.timestamp);
  }
  ros_msg.header.frame_id = frame_id_;
  ros_msg.linear_acceleration.x = From_g_To_ms2(imu_config_.imu_accel_x);
  ros_msg.linear_acceleration.y = From_g_To_ms2(imu_config_.imu_accel_y);
  ros_msg.linear_acceleration.z = From_g_To_ms2(imu_config_.imu_accel_z);
  ros_msg.angular_velocity.x = From_degs_To_rads(imu_config_.imu_ang_vel_x);
  ros_msg.angular_velocity.y = From_degs_To_rads(imu_config_.imu_ang_vel_y);
  ros_msg.angular_velocity.z = From_degs_To_rads(imu_config_.imu_ang_vel_z);
  return ros_msg;
}

inline void SourceDriver::ReceivePacket(const hesai_ros_driver::msg::UdpFrame::SharedPtr msg)
{
  for (size_t i = 0; i < msg->packets.size(); i++) {
    if(driver_ptr_->lidar_ptr_->origin_packets_buffer_.full()) std::this_thread::sleep_for(std::chrono::microseconds(10000));
    driver_ptr_->lidar_ptr_->origin_packets_buffer_.emplace_back(&msg->packets[i].data[0], msg->packets[i].size);
  }
}

inline void SourceDriver::ReceiveCorrection(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
{
  driver_ptr_->lidar_ptr_->correction_string_.resize(msg->data.size());
  std::copy(msg->data.begin(), msg->data.end(), driver_ptr_->lidar_ptr_->correction_string_.begin());
  while (1) {
    if (! driver_ptr_->lidar_ptr_->LoadCorrectionFromROSbag()) {
      break;
    }
  }
}

// Preserve the pre-upgrade ROS contract: m/s^2 and rad/s. The upstream
// 357ff14 SDK accessors return raw g and degrees/s for JT128 (protocol 1.4).
inline double SourceDriver::From_g_To_ms2(double g)
{
  return g * 9.80665;
}

inline double SourceDriver::From_degs_To_rads(double degree)
{
  return degree * M_PI / 180.0;
}
