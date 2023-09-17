/*
 * Copyright (c) 2023, John D'Angelo
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

#ifndef PROJECTED_POINT_CLOUD_TRANSPORT__PROJECTED_PUBLISHER_HPP_
#define PROJECTED_POINT_CLOUD_TRANSPORT__PROJECTED_PUBLISHER_HPP_

#include <memory>
#include <string>

#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

#include <point_cloud_interfaces/msg/organized_point_cloud.hpp>
#include <point_cloud_transport/point_cloud_transport.hpp>
#include <point_cloud_transport/simple_publisher_plugin.hpp>


namespace organized_point_cloud_transport
{

class OrganizedPublisher
  : public point_cloud_transport::SimplePublisherPlugin<
    point_cloud_interfaces::msg::OrganizedPointCloud>
{
public:
  std::string getTransportName() const override;

  void declareParameters(const std::string & base_topic) override;

  TypedEncodeResult encodeTyped(const sensor_msgs::msg::PointCloud2 & raw) const override;

  std::string getDataType() const override
  {
    return "point_cloud_interfaces/msg/OrganizedPointCloud";
  }

private:

  void encodeOrganizedPointCloud2(const sensor_msgs::msg::PointCloud2& cloud, std::vector<uint8_t>& compressed_data) const;

  void organizePointCloud2(const sensor_msgs::msg::PointCloud2& cloud, sensor_msgs::msg::PointCloud2& organized) const;

  // tf2 machinery
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // params for planar projection
  sensor_msgs::msg::CameraInfo projector_info_;

  // params for compression
  int png_level_;

};
}  // namespace organized_point_cloud_transport

#endif  // PROJECTED_POINT_CLOUD_TRANSPORT__PROJECTED_PUBLISHER_HPP_
