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

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <organized_point_cloud_transport/organized_subscriber.hpp>

namespace organized_point_cloud_transport
{
void OrganizedSubscriber::declareParameters()
{
}

std::string OrganizedSubscriber::getTransportName() const
{
  return "organized";
}

OrganizedSubscriber::DecodeResult OrganizedSubscriber::decodeTyped(
  const point_cloud_interfaces::msg::CompressedPointCloud2 & msg) const
{
  auto result = std::make_shared<sensor_msgs::msg::PointCloud2>();

  // determine if this is an xyz only or an xyzrgb cloud
  const auto predicate = [](const auto& field) { return field.name == "rgb"; };
  const auto field_result = std::find_if(msg.fields.cbegin (), msg.fields.cend (), predicate);
  const bool has_rgb = (field_result != msg.fields.cend());

  // decompress the cloud
  std::istringstream stream(reinterpret_cast<const char*>(msg.compressed_data.data()));
  if(has_rgb){
    auto temp_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    pcl::io::OrganizedPointCloudCompression<pcl::PointXYZRGB> decoder;
    decoder.decodePointCloud (stream, temp_cloud, false);
    pcl::PCLPointCloud2::Ptr temp_cloud_pcl2 = pcl::make_shared<pcl::PCLPointCloud2>();
    pcl::toPCLPointCloud2(*temp_cloud, *temp_cloud_pcl2);
    pcl_conversions::moveFromPCL(*temp_cloud_pcl2, *result);
  }else{
    auto temp_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::io::OrganizedPointCloudCompression<pcl::PointXYZ> decoder;
    decoder.decodePointCloud (stream, temp_cloud, false);
    pcl::PCLPointCloud2::Ptr temp_cloud_pcl2 = pcl::make_shared<pcl::PCLPointCloud2>();
    pcl::toPCLPointCloud2(*temp_cloud, *temp_cloud_pcl2);
    pcl_conversions::moveFromPCL(*temp_cloud_pcl2, *result);
  }
  result->header = msg.header;

  return result;
}

}  // namespace organized_point_cloud_transport
