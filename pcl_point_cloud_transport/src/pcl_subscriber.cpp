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

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

#include <memory>
#include <string>
#include <vector>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl_point_cloud_transport/pcl_subscriber.hpp>

namespace pcl_point_cloud_transport
{
void PCLSubscriber::declareParameters()
{
}

std::string PCLSubscriber::getTransportName() const
{
  return "pcl";
}

PCLSubscriber::DecodeResult PCLSubscriber::decodeTyped(
  const pcl_point_cloud_transport::PCLContainer & container) const
{
  auto message = std::make_shared<sensor_msgs::msg::PointCloud2>();

  pcl::PCLPointCloud2 pcl_pc2 = container.pcl();

  std::stringstream ss;
  ss << "output.pcd";
  std::cerr << "Data saved to " << ss.str() << std::endl;

  pcl::io::savePCDFile(
    ss.str(), pcl_pc2, Eigen::Vector4f::Zero(),
    Eigen::Quaternionf::Identity(), true);

  pcl_conversions::moveFromPCL(pcl_pc2, *message.get());

  return message;
}

}  // namespace pcl_point_cloud_transport
