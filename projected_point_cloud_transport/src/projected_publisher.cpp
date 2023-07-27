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

  // Populate the msg metadata
  compressed.projection_type = projection_type_;

  // TODO (john.dangelo@tailos.com): what if frame_id of the view point != frame id of the point cloud
  compressed.view_point = view_point_;

  compressed.was_dense = raw.is_dense;
  compressed.header = raw.header;
  // TODO (john.dangelo@tailos.com): Support the fields?
  // compressed.fields = raw.fields;

  return compressed;
}

void ProjectedPublisher::projectCloudOntoPlane(const sensor_msgs::msg::PointCloud2& cloud, cv::Mat& projected_pointcloud_image){
  if(cloud.is_dense){
    // TODO (john.dangelo@tailos.com): if the pointcloud is already organized, just point the cv::Mat at the data
    projected_pointcloud_image = cv::Mat(cv::Size(cloud.width, cloud.height), CV_16UC1, cloud.data);
  }else{
    // TODO (john.dangelo@tailos.com): Make these parameters
    const int view_height = 720;
    const int view_width = 1080;
    projected_pointcloud_image = cv::Mat(cv::Size(view_width, view_height), CV_16UC1, cv::Scalar(0));
    const float ppx = view_width/2;
    const float ppy = view_height/2;
    const float fx = view_width;
    const float fy = view_width;
    // if the pointcloud is NOT already organized, we need to apply the projection
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      const double& x = *iter_x - view_point.position.x;
      const double& y = *iter_y - view_point.position.y;
      const double& z = *iter_z - view_point.position.z;

      if(!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)){
        continue;
      }

      // TODO (john-maidbot): rotate the d x/y/z so that z is along the view point orientation
      // TODO: can that be made more efficient by transforming the view point instead?

      if(z == 0){
        continue;
      }

      const int col = x/z * fx + ppx;
      const int row = y/z * fy + ppy;

      auto& cell = projected_pointcloud_image.at<uint16_t>(row, col);
      cell = std::min(cell, static_cast<uint16_t>(depth * 1000)); // mm resolution
    }    

  }
}

void ProjectedPublisher::projectCloudOntoSphere(const sensor_msgs::msg::PointCloud2& cloud, cv::Mat& projected_pointcloud_image){
  // TODO (john.dangelo@tailos.com): Make these parameters
  const double phi_resolution = 0.5; // radians
  const double theta_resolution = 0.5; // radians
  const int phi_bins = 2 * M_PI / phi_resolution;
  const int theta_bins = 2 * M_PI / rho_resolution;

  if(spherical_image.empty()){
    spherical_image_ = cv::Mat{phi_bins, theta_bins, CV_16UC1, cv::Scalar(0)};
  }else{
    spherical_image.setTo(0);
  }

  // iterate over the pointcloud and convert to polar coordinates
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(raw, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(raw, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(raw, "z");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    // TODO (john.dangelo@tailos.com): should spherical projection account
    // for the orientation of the view point?
    const double& x = *iter_x - view_point.position.x;
    const double& y = *iter_y - view_point.position.y;
    const double& z = *iter_z - view_point.position.z;

    if(!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)){
      continue;
    }

    const double rho = std::sqrt(x * x + y * y + z * z);
    const double phi = std::atan2(y, x);
    const double theta = std::acos(z / rho);

    const int phi_index = phi / phi_resolution;
    const int theta_index = theta / theta_resolution;

    auto& cell = spherical_image_.at<uint16_t>(phi_index, rho_index);
    cell = std::min(cell, static_cast<uint16_t>(rho * 1000)); // mm resolution
  }
  projected_pointcloud_image = spherical_image_;
}

}  // namespace projected_point_cloud_transport
