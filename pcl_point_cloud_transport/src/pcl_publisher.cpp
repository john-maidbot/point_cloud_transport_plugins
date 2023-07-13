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

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <pcl_point_cloud_transport/pcl_publisher.hpp>
#include "pcl_point_cloud_transport/pcl_sensor_msgs_pointcloud2_type_adapter.hpp"

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(
  pcl_point_cloud_transport::PCLContainer,
  sensor_msgs::msg::PointCloud2);

namespace pcl_point_cloud_transport
{

void PCLPublisher::declareParameters(const std::string & /*base_topic*/)
{
}

std::string PCLPublisher::getTransportName() const
{
  return "pcl";
}

PCLPublisher::TypedEncodeResult PCLPublisher::encodeTyped(
  const sensor_msgs::msg::PointCloud2 & raw) const
{
  pcl_point_cloud_transport::PCLContainer container(raw);
  return std::optional<pcl_point_cloud_transport::PCLContainer>(container);
}

}  // namespace pcl_point_cloud_transport
