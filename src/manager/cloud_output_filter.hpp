#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

namespace hesai_ros
{
// Applied to SDK points BEFORE allocating/serializing a ROS cloud. Coordinates
// remain in the sensor frame; the static transform is used only for the crop.
struct CloudOutputFilter
{
  bool enabled = false;
  std::uint16_t stride = 1;
  bool crop_enabled = false;
  std::array<double, 3> crop_min{{0, 0, 0}}, crop_max{{0, 0, 0}};
  std::array<double, 9> rotation{{1, 0, 0, 0, 1, 0, 0, 0, 1}};
  std::array<double, 3> translation{{0, 0, 0}};
  double min_range = 0.0, max_range = 1000.0;

  void validate() const
  {
    if (!stride || !std::isfinite(min_range) || !std::isfinite(max_range) ||
        min_range < 0 || max_range <= min_range)
      throw std::invalid_argument("invalid output filter stride/range");
    for (size_t i = 0; i < 3; ++i) {
      if (!std::isfinite(crop_min[i]) || !std::isfinite(crop_max[i]) ||
          crop_min[i] > crop_max[i] || !std::isfinite(translation[i]))
        throw std::invalid_argument("invalid output crop bounds/translation");
      for (size_t j = 0; j < 3; ++j) {
        double dot = 0;
        for (size_t k = 0; k < 3; ++k) dot += rotation[3*i+k]*rotation[3*j+k];
        if (!std::isfinite(dot) || std::abs(dot - (i == j ? 1.0 : 0.0)) > 1e-5)
          throw std::invalid_argument("output crop rotation must be orthonormal");
      }
    }
    const auto &r = rotation;
    const double det = r[0]*(r[4]*r[8]-r[5]*r[7]) -
      r[1]*(r[3]*r[8]-r[5]*r[6]) + r[2]*(r[3]*r[7]-r[4]*r[6]);
    if (std::abs(det - 1) > 1e-5)
      throw std::invalid_argument("output crop rotation must be proper");
  }

  template<class Point> bool outsideCrop(const Point &p) const
  {
    if (!crop_enabled) return true;
    // Float arithmetic agrees with the former PCL CropBox / URDF transform.
    const float v[3] = {p.x, p.y, p.z};
    for (size_t i = 0; i < 3; ++i) {
      float b = static_cast<float>(translation[i]);
      for (size_t j = 0; j < 3; ++j) b += static_cast<float>(rotation[3*i+j])*v[j];
      if (b < static_cast<float>(crop_min[i]) || b > static_cast<float>(crop_max[i]))
        return true;
    }
    return false;  // inclusive box, as in the former negative CropBox
  }
};

struct CloudOutputStats
{
  size_t input_points = 0, crop_removed = 0, selected_points = 0;
  size_t invalid_points = 0, range_removed = 0;
  bool timing_point = false;
  double scan_end = 0;
};

inline sensor_msgs::msg::PointField cloudField(
  const std::string &name, uint32_t offset, uint8_t type)
{
  sensor_msgs::msg::PointField f;
  f.name = name; f.offset = offset; f.datatype = type; f.count = 1;
  return f;
}

template<class T> void putScalar(std::vector<uint8_t> &data, size_t offset, const T &value)
{
  // Hesai's timestamp is intentionally unaligned (offset 18).
  std::memcpy(data.data() + offset, &value, sizeof(T));
}

template<class Point>
sensor_msgs::msg::PointCloud2 makeOutputCloud(
  const Point *points, size_t count, double start, const std::string &frame,
  const CloudOutputFilter &filter, CloudOutputStats &stats)
{
  using Field = sensor_msgs::msg::PointField;
  if (!std::isfinite(start) || start < 0 || start >= std::numeric_limits<int32_t>::max())
    throw std::invalid_argument("invalid point cloud start timestamp");
  if (filter.enabled && filter.stride == 0)
    throw std::invalid_argument("output filter stride must be positive");
  stats = CloudOutputStats{};
  stats.input_points = count;
  stats.scan_end = start;
  std::vector<size_t> selected;
  selected.reserve(count / (filter.enabled ? filter.stride : 1) + 1);
  size_t cropped_index = 0;
  double retained_end = start;
  const double min_sq = filter.min_range*filter.min_range;
  const double max_sq = filter.max_range*filter.max_range;
  for (size_t i = 0; i < count; ++i) {
    const auto &p = points[i];
    if (filter.enabled) {
      // Validate every timestamp, including removed points. Never conceal bad
      // timing by decimating it away. Matches FAST-LIO's 0.1 ms tolerance.
      if (!std::isfinite(p.timestamp) || p.timestamp < start - 0.0001 ||
          p.timestamp > start + 0.2)
        throw std::invalid_argument("point timestamp outside the 200 ms scan interval");
      stats.scan_end = std::max(stats.scan_end, p.timestamp);
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
        ++stats.invalid_points; continue;
      }
      if (!filter.outsideCrop(p)) { ++stats.crop_removed; continue; }
      // Legacy order: crop -> index decimation -> range check. Counting before
      // range filtering preserves the previous estimator's selected samples.
      if (cropped_index++ % filter.stride != 0) continue;
      if (!std::isfinite(p.intensity)) { ++stats.invalid_points; continue; }
      const double d2 = double(p.x)*p.x + double(p.y)*p.y + double(p.z)*p.z;
      if (d2 < min_sq || d2 > max_sq) { ++stats.range_removed; continue; }
    }
    if (std::isfinite(p.timestamp)) stats.scan_end = std::max(stats.scan_end, p.timestamp);
    selected.push_back(i);
    retained_end = std::max(retained_end, p.timestamp);
  }
  stats.selected_points = selected.size();
  // PointCloud2 has no scan-end header field. A single invalid XYZ record
  // preserves the original end timestamp when the last firing was removed.
  // It is timing only, never an obstacle or an estimator feature. FAST-LIO
  // reads all timestamps before rejecting invalid coordinates.
  stats.timing_point = filter.enabled && !selected.empty() && stats.scan_end > retained_end;
  sensor_msgs::msg::PointCloud2 msg;
  const double seconds = std::floor(start);
  msg.header.stamp.sec = static_cast<int32_t>(seconds);
  msg.header.stamp.nanosec = static_cast<uint32_t>(std::llround((start - seconds) * 1e9));
  if (msg.header.stamp.nanosec >= 1000000000U) {
    ++msg.header.stamp.sec;
    msg.header.stamp.nanosec -= 1000000000U;
  }
  msg.header.frame_id = frame;
  msg.height = 1;
  msg.width = static_cast<uint32_t>(selected.size() + size_t(stats.timing_point));
  msg.fields = {cloudField("x",0,Field::FLOAT32), cloudField("y",4,Field::FLOAT32),
    cloudField("z",8,Field::FLOAT32), cloudField("intensity",12,Field::FLOAT32),
    cloudField("ring",16,Field::UINT16), cloudField("timestamp",18,Field::FLOAT64)};
  msg.point_step = 26;
  if (filter.enabled) {
    // Self-describing compact bags: consumers can avoid a second decimation.
    msg.fields.push_back(cloudField("preprocessing_stride",26,Field::UINT16));
    msg.point_step = 28;
  }
  msg.is_dense = false;
  msg.is_bigendian = false;
  msg.row_step = msg.width * msg.point_step;
  msg.data.resize(msg.row_step);
  for (size_t i = 0; i < selected.size(); ++i) {
    const auto &p = points[selected[i]];
    const size_t b = i * msg.point_step;
    putScalar<float>(msg.data,b,p.x); putScalar<float>(msg.data,b+4,p.y);
    putScalar<float>(msg.data,b+8,p.z); putScalar<float>(msg.data,b+12,p.intensity);
    putScalar<uint16_t>(msg.data,b+16,p.ring); putScalar<double>(msg.data,b+18,p.timestamp);
    if (filter.enabled) putScalar<uint16_t>(msg.data,b+26,filter.stride);
  }
  if (stats.timing_point) {
    const size_t b = selected.size() * msg.point_step;
    const float nan = std::numeric_limits<float>::quiet_NaN();
    putScalar<float>(msg.data,b,nan); putScalar<float>(msg.data,b+4,nan);
    putScalar<float>(msg.data,b+8,nan); putScalar<double>(msg.data,b+18,stats.scan_end);
    putScalar<uint16_t>(msg.data,b+26,filter.stride);
  }
  return msg;
}
}  // namespace hesai_ros
