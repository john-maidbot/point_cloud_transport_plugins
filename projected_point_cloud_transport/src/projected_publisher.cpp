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

#include <opencv2/imgcodecs.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <projected_point_cloud_transport/projected_publisher.hpp>

namespace projected_point_cloud_transport
{

void ProjectedPublisher::declareParameters(const std::string & base_topic)
{
  // params for planar projection
  view_height_ = 720;
  view_width_ = 1080;
  ppx_ = static_cast<float>(view_width_)/2.0;
  ppy_ = static_cast<float>(view_height_)/2.0;
  fx_ = view_width_;
  fy_ = view_width_;

  // params for spherical projection
  phi_resolution_ = 0.034; // radians
  theta_resolution_ = 0.034; // radians
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
      RCLCPP_ERROR_STREAM(getLogger(), "Projection type " << projection_type_ << " is not known/supported!");
  }

  if(projected_pointcloud_image.empty()){
    RCLCPP_ERROR_STREAM(getLogger(), "Projection type " << projection_type_ << " failed to project pointcloud!");
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

void ProjectedPublisher::projectCloudOntoPlane(const sensor_msgs::msg::PointCloud2& cloud, cv::Mat& projected_pointcloud_image) const{
  if(cloud.is_dense){
    // TODO (john.dangelo@tailos.com): if the pointcloud is already organized, just point the cv::Mat at the data
    projected_pointcloud_image = cv::Mat((int)cloud.height, (int)cloud.width, CV_32FC3, (void*)cloud.data.data());
  }else{
    // if the pointcloud is NOT already organized, we need to apply the projection
    projected_pointcloud_image = cv::Mat(cv::Size(view_width_, view_height_), CV_16UC1, cv::Scalar(0));

    // iterate over the pointcloud
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      const double& x = *iter_x - view_point_.position.x;
      const double& y = *iter_y - view_point_.position.y;
      const double& z = *iter_z - view_point_.position.z;

      if(!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)){
        continue;
      }

      // TODO (john-maidbot): rotate the d x/y/z so that z is along the view point orientation
      // TODO: can that be made more efficient by transforming the view point instead?

      const int col = x/z * fx_ + ppx_;
      const int row = y/z * fy_ + ppy_;

      if(z == 0 || col < 0 || row < 0 || col >= view_width_ || row >= view_height_){
        continue;
      }

      auto& cell = projected_pointcloud_image.at<uint16_t>(row, col);
      cell = std::min(cell, static_cast<uint16_t>(z * 1000)); // mm resolution
    }    
  }
}

void ProjectedPublisher::projectCloudOntoSphere(const sensor_msgs::msg::PointCloud2& cloud, cv::Mat& projected_pointcloud_image) const{
  // compute image size based on resolution
  const int phi_bins = 2 * M_PI / phi_resolution_;
  const int theta_bins = 2 * M_PI / theta_resolution_;

  cv::Mat spherical_image{phi_bins, theta_bins, CV_16UC1, cv::Scalar(0)};

  // iterate over the pointcloud and convert to polar coordinates
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    // TODO (john.dangelo@tailos.com): should spherical projection account
    // for the orientation of the view point?
    const double& x = *iter_x - view_point_.position.x;
    const double& y = *iter_y - view_point_.position.y;
    const double& z = *iter_z - view_point_.position.z;

    if(!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)){
      continue;
    }

    const double rho = std::sqrt(x * x + y * y + z * z);
    double phi = std::atan2(y, x);
    double theta = std::acos(z / rho);
    if (phi < 0){
      phi += 2*M_PI;
    }
    if (theta < 0){
      theta += 2*M_PI;
    }

    const int phi_index = std::min(static_cast<int>(phi / phi_resolution_), phi_bins);
    const int theta_index = std::min(static_cast<int>(theta / theta_resolution_), theta_bins);

    auto& cell = spherical_image.at<uint16_t>(phi_index, theta_index);
    cell = std::min(cell, static_cast<uint16_t>(rho * 1000)); // mm resolution
  }
  projected_pointcloud_image = spherical_image;
}

}  // namespace projected_point_cloud_transport
