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

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud_iterator.hpp>

#include <projected_point_cloud_transport/projected_publisher.hpp>

namespace projected_point_cloud_transport
{

void ProjectedPublisher::declareParameters(const std::string & base_topic)
{
}

std::string ProjectedPublisher::getTransportName() const
{
  return "projected";
}

ProjectedPublisher::TypedEncodeResult ProjectedPublisher::encodeTyped(
  const sensor_msgs::msg::PointCloud2 & raw) const
{
  point_cloud_interfaces::msg::ProjectedPointCloud compressed;

  // apply selected projection
  cv::Mat projected_pointcloud_image;
  switch(projection_type_){
    case(point_cloud_interfaces::msg::ProjectedPointCloud::PINHOLE_PROJECTION):
      projectCloudOntoPlane(raw, projected_pointcloud_image);
    case(point_cloud_interfaces::msg::ProjectedPointCloud::SPHERICAL_PROJECTION):
     projectCloudOntoSphere(raw, projected_pointcloud_image);
    default:
      RCLCPP_ERROR(getLogger(), "Projection type " << projection_type_ << " is not known/supported!");
  }

  if(projected_pointcloud_image.empty()){
    RCLCPP_ERROR(getLogger(), "Projection type " << projection_type_ << " failed to project pointcloud!");
    return cras::make_unexpected("Failed to project pointcloud onto image!");
  }

  // Apply png compression
  cv::imencode(".png", projected_pointcloud_image, compressed.compressed_data);

  compressed.projection_type = projection_type_;

  compressed.view_point = view_point_;

  compressed.width = projected_pointcloud_image.cols;
  compressed.height = projected_pointcloud_image.rows;
  compressed.row_step = projected_pointcloud_image.step;
  compressed.is_bigendian = raw.is_bigendian;
  compressed.is_dense = true;
  compressed.header = raw.header;
  // TODO (john.dangelo@tailos.com): Support the fields?
  // compressed.fields = raw.fields;

  return compressed;
}

void ProjectedPublisher::projectCloudOntoPlane(const sensor_msgs::msg::PointCloud2& cloud, cv::Mat& projected_pointcloud_image){
  if(cloud.is_dense){
    // if the pointcloud is already organized, just point the cv::Mat at the data

  }else{
    // if the pointcloud is NOT already organized, we need to apply the projection


  }
}

void ProjectedPublisher::projectCloudOntoSphere(const sensor_msgs::msg::PointCloud2& cloud, cv::Mat& projected_pointcloud_image){
  
}

}  // namespace projected_point_cloud_transport
