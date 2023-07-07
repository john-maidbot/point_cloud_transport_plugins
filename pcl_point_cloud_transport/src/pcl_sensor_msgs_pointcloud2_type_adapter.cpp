// Copyright 2021 Open Source Robotics Foundation, Inc.
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
// int
// encoding2mat_type(const std::string & encoding)
// {
// }

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

  // storage_(std::move(unique_sensor_msgs_point_cloud2))
{
  // pcl::PCLPointCloud2 pcl_pc2;
  // pcl_conversions::copyPointCloud2MetaData(*unique_sensor_msgs_point_cloud2.get(), pcl_pc2); // Like pcl_conversions::toPCL, but does not copy the binary data
  // pcl::MsgFieldMap field_map;
  // pcl::createMapping<T> (pcl_pc2.fields, field_map);
  // pcl::fromPCLPointCloud2(pcl_pc2, frame_, field_map, &(*unique_sensor_msgs_point_cloud2.get(),).data[0]);

  pcl_conversions::toPCL(*unique_sensor_msgs_point_cloud2.get(), frame_);

  storage_ = std::move(unique_sensor_msgs_point_cloud2);

  //
  // pcl::copyPointCloud(other.frame_, frame_);

}

PCLContainer::PCLContainer(
  std::shared_ptr<sensor_msgs::msg::PointCloud2> shared_sensor_msgs_point_cloud2)
: header_(shared_sensor_msgs_point_cloud2->header)
//   frame_(
//     shared_sensor_msgs_point_cloud2->height,
//     shared_sensor_msgs_point_cloud2->width,
//     encoding2mat_type(shared_sensor_msgs_point_cloud2->encoding),
//     shared_sensor_msgs_point_cloud2->data.data(),
//     shared_sensor_msgs_point_cloud2->step),
//   storage_(shared_sensor_msgs_point_cloud2)
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
  // sensor_msgs_point_cloud2.height = frame_.rows;
  // sensor_msgs_point_cloud2.width = frame_.cols;
  // switch (frame_.type()) {
  //   case CV_8UC1:
  //     sensor_msgs_point_cloud2.encoding = "mono8";
  //     break;
  //   case CV_8UC3:
  //     sensor_msgs_point_cloud2.encoding = "bgr8";
  //     break;
  //   case CV_16SC1:
  //     sensor_msgs_point_cloud2.encoding = "mono16";
  //     break;
  //   case CV_8UC4:
  //     sensor_msgs_point_cloud2.encoding = "rgba8";
  //     break;
  //   default:
  //     throw std::runtime_error("unsupported encoding type");
  // }
  // sensor_msgs_point_cloud2.step = static_cast<sensor_msgs::msg::PointCloud2::_step_type>(frame_.step);
  // size_t size = frame_.step * frame_.rows;
  // sensor_msgs_point_cloud2.data.resize(size);
  // memcpy(&sensor_msgs_point_cloud2.data[0], frame_.data, size);
  // sensor_msgs_point_cloud2.header = header_;
}

bool
PCLContainer::is_bigendian() const
{
  return is_bigendian_;
}

}  // namespace image_tools
