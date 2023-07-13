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

#ifndef PCL_POINT_CLOUD_TRANSPORT__PCL_PUBLISHER_HPP_
#define PCL_POINT_CLOUD_TRANSPORT__PCL_PUBLISHER_HPP_

#include <memory>
#include <string>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <point_cloud_transport/point_cloud_transport.hpp>
#include "pcl_point_cloud_transport/pcl_sensor_msgs_pointcloud2_type_adapter.hpp"

#include <point_cloud_transport/simple_publisher_plugin.hpp>


namespace pcl_point_cloud_transport
{

class PCLPublisher
  : public point_cloud_transport::SimplePublisherPlugin<
    pcl_point_cloud_transport::PCLContainer>
{
public:
  std::string getTransportName() const override;

  void declareParameters(const std::string & base_topic) override;

  TypedEncodeResult encodeTyped(const sensor_msgs::msg::PointCloud2 & raw) const override;
};
}  // namespace pcl_point_cloud_transport

#endif  // PCL_POINT_CLOUD_TRANSPORT__PCL_PUBLISHER_HPP_
