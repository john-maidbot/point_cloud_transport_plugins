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

#include <string>
#include <sstream>

#include <pcl-1.12/pcl/compression/organized_pointcloud_compression.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <organized_point_cloud_transport/organized_publisher.hpp>

namespace organized_point_cloud_transport
{

  void OrganizedPublisher::declareParameters(const std::string &base_topic)
  {
    // params for planar projection
    projector_info_.height = 720;
    projector_info_.width = 1080;
    projector_info_.k[2] = static_cast<float>(projector_info_.width) / 2.0;
    projector_info_.k[5] = static_cast<float>(projector_info_.height) / 2.0;
    const float fov = M_PI_2;
    projector_info_.k[0] = projector_info_.width / (2 * std::tan(fov / 2.0));
    projector_info_.k[4] = projector_info_.height / (2 * std::tan(fov / 2.0));
    // compression params
    png_level_ = 3;

    // if this is the first time receieving a message, setup the tf2 machinery
    if (!tf_buffer_)
    {
      auto node_ptr = getNode();
      if (node_ptr == nullptr)
      {
        RCLCPP_ERROR_STREAM(getLogger(), "Node pointer is null!");
      }
      tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_ptr->get_clock());
      // disable intra process communication in case someone is using this in a composition
      rclcpp::SubscriptionOptions tf2_subscription_options;
      tf2_subscription_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
          *tf_buffer_, node_ptr, true, tf2_ros::DynamicListenerQoS(), tf2_ros::StaticListenerQoS(),
          tf2_subscription_options, tf2_subscription_options);
    }
  }

  std::string OrganizedPublisher::getTransportName() const
  {
    return "organized";
  }

  OrganizedPublisher::TypedEncodeResult OrganizedPublisher::encodeTyped(
      const sensor_msgs::msg::PointCloud2 &raw) const
  {
    point_cloud_interfaces::msg::CompressedPointCloud2 compressed;

    if (raw.is_dense)
    {
      encodeOrganizedPointCloud2(raw, compressed.compressed_data);
      compressed.header = raw.header;
      compressed.height = raw.height;
      compressed.width = raw.width;
      compressed.fields = raw.fields;
      compressed.is_bigendian = raw.is_bigendian;
    }
    else
    {
      sensor_msgs::msg::PointCloud2 organized;
      organizePointCloud2(raw, organized);
      encodeOrganizedPointCloud2(organized, compressed.compressed_data);
      // Populate the msg metadata
      compressed.header = organized.header;
      compressed.height = organized.height;
      compressed.width = organized.width;
      compressed.fields = organized.fields;
      compressed.is_bigendian = organized.is_bigendian;
    }
    compressed.format = getTransportName();

    return compressed;
  }

  void OrganizedPublisher::encodeOrganizedPointCloud2(const sensor_msgs::msg::PointCloud2 &cloud, std::vector<uint8_t> &compressed_data) const
  {
    const auto predicate = [](const auto &field)
    { return field.name == "rgb"; };
    const auto result = std::find_if(cloud.fields.cbegin(), cloud.fields.cend(), predicate);
    const bool has_rgb = result != cloud.fields.cend();

    // PCL has a nice solution for organized pointclouds, so let's not reinvent the wheel
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(cloud, pcl_pc2);
    std::ostringstream compressed_data_stream;
    if (has_rgb)
    {
      auto temp_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
      pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);
      pcl::io::OrganizedPointCloudCompression<pcl::PointXYZRGB> encoder;
      encoder.encodePointCloud(temp_cloud,
                               compressed_data_stream,
                               true,
                               false,
                               false,
                               png_level_);
    }
    else
    {
      auto temp_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
      pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);
      pcl::io::OrganizedPointCloudCompression<pcl::PointXYZ> encoder;
      encoder.encodePointCloud(temp_cloud,
                               compressed_data_stream,
                               false,
                               false,
                               false,
                               png_level_);
    }
    compressed_data = std::vector<uint8_t>(compressed_data_stream.str().begin(), compressed_data_stream.str().end());
  }

} // namespace organized_point_cloud_transport
