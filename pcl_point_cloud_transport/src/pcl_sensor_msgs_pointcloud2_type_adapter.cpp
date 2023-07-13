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

#include <cstddef>
#include <memory>
#include <string>
#include <utility>
#include <variant>  // NOLINT[build/include_order]

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

#include "pcl_point_cloud_transport/pcl_sensor_msgs_pointcloud2_type_adapter.hpp"

namespace pcl_point_cloud_transport
{

namespace
{
template<typename T>
struct NotNull
{
  NotNull(const T * pointer_in, const char * msg)
  : pointer(pointer_in)
  {
    if (pointer == nullptr) {
      throw std::invalid_argument(msg);
    }
  }

  const T * pointer;
};

}  // namespace

PCLContainer::PCLContainer(
  std::unique_ptr<sensor_msgs::msg::PointCloud2> unique_sensor_msgs_point_cloud2)
: header_(NotNull(
      unique_sensor_msgs_point_cloud2.get(),
      "unique_sensor_msgs_point_cloud2 cannot be nullptr"
).pointer->header)
{
  pcl_conversions::toPCL(*unique_sensor_msgs_point_cloud2.get(), frame_);
  storage_ = std::move(unique_sensor_msgs_point_cloud2);
}

PCLContainer::PCLContainer(
  std::shared_ptr<sensor_msgs::msg::PointCloud2> shared_sensor_msgs_point_cloud2)
: header_(shared_sensor_msgs_point_cloud2->header)
{
  pcl_conversions::toPCL(*shared_sensor_msgs_point_cloud2.get(), frame_);
  storage_ = shared_sensor_msgs_point_cloud2;
}

PCLContainer::PCLContainer(
  const pcl::PCLPointCloud2 & pcl_frame,
  const std_msgs::msg::Header & header,
  bool is_bigendian)
: header_(header),
  frame_(pcl_frame),
  storage_(nullptr),
  is_bigendian_(is_bigendian)
{}

PCLContainer::PCLContainer(
  pcl::PCLPointCloud2 && pcl_frame,
  const std_msgs::msg::Header & header,
  bool is_bigendian)
: header_(header),
  frame_(std::forward<pcl::PCLPointCloud2>(pcl_frame)),
  storage_(nullptr),
  is_bigendian_(is_bigendian)
{}

PCLContainer::PCLContainer(
  const sensor_msgs::msg::PointCloud2 & sensor_msgs_point_cloud2)
: PCLContainer(std::make_unique<sensor_msgs::msg::PointCloud2>(sensor_msgs_point_cloud2))
{}

bool
PCLContainer::is_owning() const
{
  return std::holds_alternative<std::nullptr_t>(storage_);
}

const pcl::PCLPointCloud2 &
PCLContainer::pcl() const
{
  return frame_;
}

pcl::PCLPointCloud2
PCLContainer::pcl()
{
  return frame_;
}

const std_msgs::msg::Header &
PCLContainer::header() const
{
  return header_;
}

std_msgs::msg::Header &
PCLContainer::header()
{
  return header_;
}

std::shared_ptr<const sensor_msgs::msg::PointCloud2>
PCLContainer::get_sensor_msgs_msg_point_cloud2_pointer() const
{
  if (!std::holds_alternative<std::shared_ptr<sensor_msgs::msg::PointCloud2>>(storage_)) {
    return nullptr;
  }
  return std::get<std::shared_ptr<sensor_msgs::msg::PointCloud2>>(storage_);
}

std::unique_ptr<sensor_msgs::msg::PointCloud2>
PCLContainer::get_sensor_msgs_msg_point_cloud2_pointer_copy() const
{
  auto unique_image = std::make_unique<sensor_msgs::msg::PointCloud2>();
  this->get_sensor_msgs_msg_point_cloud2_copy(*unique_image);
  return unique_image;
}

void
PCLContainer::get_sensor_msgs_msg_point_cloud2_copy(
  sensor_msgs::msg::PointCloud2 & sensor_msgs_point_cloud2) const
{
  // TODO(ahcorde): Implement this emthod
}

bool
PCLContainer::is_bigendian() const
{
  return is_bigendian_;
}

}  // namespace pcl_point_cloud_transport
