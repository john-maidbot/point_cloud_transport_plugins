// Copyright 2023 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef IMAGE_TOOLS__CV_MAT_sensor_msgs_point_cloud2_TYPE_ADAPTER_HPP_
#define IMAGE_TOOLS__CV_MAT_sensor_msgs_point_cloud2_TYPE_ADAPTER_HPP_

#include <cstddef>
#include <memory>
#include <variant>  // NOLINT[build/include_order]

#include <pcl/common/io.h>
#include <pcl/PCLPointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/type_adapter.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace pcl_point_cloud_transport
{
namespace detail
{
// TODO(audrow): Replace with std::endian when C++ 20 is available
// https://en.cppreference.com/w/cpp/types/endian
enum class endian
{
#ifdef _WIN32
  little = 0,
  big    = 1,
  native = little
#else
  little = __ORDER_LITTLE_ENDIAN__,
  big    = __ORDER_BIG_ENDIAN__,
  native = __BYTE_ORDER__
#endif
};

}  // namespace detail

class PCLContainer
{
  static constexpr bool is_bigendian_system = detail::endian::native == detail::endian::big;

public:
  using PointCloud2MsgsImageStorageType = std::variant<
    std::nullptr_t,
    std::unique_ptr<sensor_msgs::msg::PointCloud2>,
    std::shared_ptr<sensor_msgs::msg::PointCloud2>
  >;

  PCLContainer() = default;

  explicit PCLContainer(const PCLContainer & other)
  : header_(other.header_), frame_(other.frame_), is_bigendian_(other.is_bigendian_)
  {
    if (std::holds_alternative<std::shared_ptr<sensor_msgs::msg::PointCloud2>>(other.storage_)) {
      storage_ = std::get<std::shared_ptr<sensor_msgs::msg::PointCloud2>>(other.storage_);
    } else if (std::holds_alternative<std::unique_ptr<sensor_msgs::msg::PointCloud2>>(other.storage_)) {
      storage_ = std::make_unique<sensor_msgs::msg::PointCloud2>(
        *std::get<std::unique_ptr<sensor_msgs::msg::PointCloud2>>(other.storage_));
    }
  }

  PCLContainer & operator=(const PCLContainer & other)
  {
    if (this != &other) {
      header_ = other.header_;
      pcl::copyPointCloud(other.frame_, frame_);
      is_bigendian_ = other.is_bigendian_;
      if (std::holds_alternative<std::shared_ptr<sensor_msgs::msg::PointCloud2>>(other.storage_)) {
        storage_ = std::get<std::shared_ptr<sensor_msgs::msg::PointCloud2>>(other.storage_);
      } else if (std::holds_alternative<std::unique_ptr<sensor_msgs::msg::PointCloud2>>(other.storage_)) {
        storage_ = std::make_unique<sensor_msgs::msg::PointCloud2>(
          *std::get<std::unique_ptr<sensor_msgs::msg::PointCloud2>>(other.storage_));
      } else if (std::holds_alternative<std::nullptr_t>(other.storage_)) {
        storage_ = nullptr;
      }
    }
    return *this;
  }

  /// Store an owning pointer to a sensor_msg::msg::PointCloud2, and create a pcl::PCLPointCloud2 that references it.
  explicit PCLContainer(std::unique_ptr<sensor_msgs::msg::PointCloud2> unique_sensor_msgs_point_cloud2);

  /// Store an owning pointer to a sensor_msg::msg::PointCloud2, and create a pcl::PCLPointCloud2 that references it.
  explicit PCLContainer(std::shared_ptr<sensor_msgs::msg::PointCloud2> shared_sensor_msgs_point_cloud2);

  /// Shallow copy the given pcl::PCLPointCloud2 into this class, but do not own the data directly.
  PCLContainer(
    const pcl::PCLPointCloud2 & pcl_frame,
    const std_msgs::msg::Header & header,
    bool is_bigendian = is_bigendian_system);

  /// Move the given pcl::PCLPointCloud2 into this class.
  PCLContainer(
    pcl::PCLPointCloud2 && pcl_frame,
    const std_msgs::msg::Header & header,
    bool is_bigendian = is_bigendian_system);

  /// Copy the sensor_msgs::msg::PointCloud2 into this contain and create a pcl::PCLPointCloud2 that references it.
  explicit PCLContainer(const sensor_msgs::msg::PointCloud2 & sensor_msgs_point_cloud2);

  /// Return true if this class owns the data the cv_mat references.
  /**
   * Note that this does not check if the pcl::PCLPointCloud2 owns its own data, only if
   * this class owns a sensor_msgs::msg::PointCloud2 that the pcl::PCLPointCloud2 references.
   */
  bool
  is_owning() const;

  /// Const access the pcl::PCLPointCloud2 in this class.
  const pcl::PCLPointCloud2 &
  pcl() const;

  /// Get a shallow copy of the pcl::PCLPointCloud2 that is in this class.
  /**
   * Note that if you want to let this container go out of scope you should
   * make a deep copy with pcl::PCLPointCloud2::clone() beforehand.
   */
  pcl::PCLPointCloud2
  pcl();

  /// Const access the ROS Header.
  const std_msgs::msg::Header &
  header() const;

  /// Access the ROS Header.
  std_msgs::msg::Header &
  header();

  /// Get shared const pointer to the sensor_msgs::msg::PointCloud2 if available, otherwise nullptr.
  std::shared_ptr<const sensor_msgs::msg::PointCloud2>
  get_sensor_msgs_msg_point_cloud2_pointer() const;

  /// Get copy as a unique pointer to the sensor_msgs::msg::PointCloud2.
  std::unique_ptr<sensor_msgs::msg::PointCloud2>
  get_sensor_msgs_msg_point_cloud2_pointer_copy() const;

  /// Get a copy of the image as a sensor_msgs::msg::PointCloud2.
  sensor_msgs::msg::PointCloud2
  get_sensor_msgs_msg_point_cloud2_copy() const;

  /// Get a copy of the image as a sensor_msgs::msg::PointCloud2.
  void
  get_sensor_msgs_msg_point_cloud2_copy(sensor_msgs::msg::PointCloud2 & sensor_msgs_point_cloud2) const;

  /// Return true if the data is stored in big endian, otherwise return false.
  bool
  is_bigendian() const;

private:
  std_msgs::msg::Header header_;
  pcl::PCLPointCloud2 frame_;
  PointCloud2MsgsImageStorageType storage_;
  bool is_bigendian_;
};

}  // namespace image_tools

template<>
struct rclcpp::TypeAdapter<pcl_point_cloud_transport::PCLContainer, sensor_msgs::msg::PointCloud2>
{
  using is_specialized = std::true_type;
  using custom_type = pcl_point_cloud_transport::PCLContainer;
  using ros_message_type = sensor_msgs::msg::PointCloud2;

  static
  void
  convert_to_ros_message(
    const custom_type & source,
    ros_message_type & destination)
  {
    sensor_msgs::msg::PointCloud2 msg;
    pcl::PCLPointCloud2 cloud = source.pcl();
    // pcl::toROSMsg<pcl::PCLPointCloud2>(cloud, msg);
    pcl_conversions::moveFromPCL(cloud, destination);


    // destination.height = source.cv_mat().rows;
    // destination.width = source.cv_mat().cols;
    // switch (source.cv_mat().type()) {
    //   case CV_8UC1:
    //     destination.encoding = "mono8";
    //     break;
    //   case CV_8UC3:
    //     destination.encoding = "bgr8";
    //     break;
    //   case CV_16SC1:
    //     destination.encoding = "mono16";
    //     break;
    //   case CV_8UC4:
    //     destination.encoding = "rgba8";
    //     break;
    //   default:
    //     throw std::runtime_error("unsupported encoding type");
    // }
    // destination.step = static_cast<sensor_msgs::msg::PointCloud2::_step_type>(source.cv_mat().step);
    // size_t size = source.cv_mat().step * source.cv_mat().rows;
    // destination.data.resize(size);
    // memcpy(&destination.data[0], source.cv_mat().data, size);
    // destination.header = source.header();
  }

  static
  void
  convert_to_custom(
    const ros_message_type & source,
    custom_type & destination)
  {
    destination = pcl_point_cloud_transport::PCLContainer(source);
  }
};

#endif  // IMAGE_TOOLS__CV_MAT_sensor_msgs_point_cloud2_TYPE_ADAPTER_HPP_
