/*
 * Copyright (c) 2023, Czech Technical University in Prague
 * Copyright (c) 2019, paplhjak
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *
 *    * Neither the name of the copyright holder nor the names of its
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

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

void PCLPublisher::declareParameters(const std::string & base_topic)
{
}

// cras::expected<std::unique_ptr<pcl_point_cloud_transport::PCLContainer>, std::string> PCLPublisher::convertPC2toPCL(
//   const sensor_msgs::msg::PointCloud2 & PC2, const std::string & topic, bool deduplicate,
//   bool expert_encoding) const
// {
//   // std_msgs::msg::Header header;
// // header.frame_id = frame_id_;
// // header.stamp = this->now();
//   // pcl_point_cloud_transport::PCLContainer container(PC2);
//   // return container.get_sensor_msgs_msg_point_cloud2_pointer_copy();
// }

std::string PCLPublisher::getTransportName() const
{
  return "pcl";
}

PCLPublisher::TypedEncodeResult PCLPublisher::encodeTyped(
  const sensor_msgs::msg::PointCloud2 & raw) const
{
  pcl_point_cloud_transport::PCLContainer container(raw);
  return std::optional<pcl_point_cloud_transport::PCLContainer>(container);
  // cras::make_unexpected("lol");
}

}  // namespace draco_point_cloud_transport
