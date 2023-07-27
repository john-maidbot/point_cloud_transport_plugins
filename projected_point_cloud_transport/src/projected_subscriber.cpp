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
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/point_cloud2_modifier.hpp>

#include <projected_point_cloud_transport/projected_subscriber.hpp>

namespace projected_point_cloud_transport
{
void ProjectedSubscriber::declareParameters()
{
}

std::string ProjectedSubscriber::getTransportName() const
{
  return "projected";
}

ProjectedSubscriber::DecodeResult ProjectedSubscriber::decodeTyped(
  const point_cloud_interfaces::msg::ProjectedPointCloud & msg) const
{
  auto result = std::make_shared<sensor_msgs::msg::PointCloud2>();

  cv::Mat projected_pointcloud_image = cv::imdecode(cv::Mat(msg.compressed_data), cv::IMREAD_UNCHANGED);

  switch(msg.projection_type){
    case(point_cloud_interfaces::msg::ProjectedPointCloud::PINHOLE_PROJECTION):
      deprojectPlaneToCloud(projected_pointcloud_image, result);
    case(point_cloud_interfaces::msg::ProjectedPointCloud::SPHERICAL_PROJECTION):
     deprojectSphereToCloud(projected_pointcloud_image, result);
    default:
      RCLCPP_ERROR(getLogger(), "Projection type " << projection_type_ << " is not known/supported!");
  }

  if(projected_pointcloud_image.empty()){
    RCLCPP_ERROR(getLogger(), "Projection type " << projection_type_ << " failed to project pointcloud!");
    return cras::make_unexpected("Failed to project pointcloud onto image!");
  }

  return result;
}

void ProjectedPublisher::deprojectPlaneToCloud(const cv::Mat& projected_pointcloud_image, const bool was_dense, sensor_msgs::msg::PointCloud2& cloud){
  if(was_dense){
    cloud.data = projected_pointcloud_image.data();
  }else{
    const int view_height = projected_pointcloud_image.rows;
    const int view_width = projected_pointcloud_image.cols;
    const float ppx = view_width/2;
    const float ppy = view_height/2;
    const float fx = view_width;
    const float fy = view_width;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2Fields(3, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
                                  sensor_msgs::msg::PointField::FLOAT32, "z", 1,
                                  sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(projected_pointcloud_image.rows * projected_pointcloud_image.cols);
    sensor_msgs::PointCloud2Iterator<float> pcl_itr(cloud, "x");

    size_t valid_points = 0;

    // if the pointcloud is NOT already organized, we need to apply the projection
    for () {
      for () {
        const uint16_t& depth = projected_pointcloud_image.at<uint16_t>(row, col);

        if(depth == 0){
          continue;
        }

        float x = depth * (col - ppx ) / fx;
        float y = depth * (row - ppy ) / fy;
        float z = depth;

        // TODO (john-maidbot): unrotate the d x/y/z based on view_point orientation

        pcl_iter[0] = x;
        pcl_iter[1] = y;
        pcl_iter[2] = z;
        ++pcl_itr;
        valid_points++;
      }
    }
    modifier.resize(valid_points);
  }
}

void ProjectedSubscriber::deprojectSphereToCloud(){
  const cv::Mat spherical_image       = cv::imdecode(cv::Mat(msg.compressed_data), cv::IMREAD_UNCHANGED);
  int phi_bins = spherical_image.rows;
  int theta_bins = spherical_image.cols;
  double phi_resolution = phi_bins / 2.0 / M_PI; // radians
  double theta_resolution = theta_bins / 2.0 / M_PI; // radians

  cloud.header = msg.header;

  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(3, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
                                sensor_msgs::msg::PointField::FLOAT32, "z", 1,
                                sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(spherical_image.rows * spherical_image.cols);
  sensor_msgs::PointCloud2Iterator<float> pcl_itr(cloud, "x");

  size_t valid_points = 0;
  // convert the spherical image to a point cloud
  for(int row = 0; row < spherical_image.rows; row++)
  {
    for(int col = 0; col < spherical_image.cols; col++)
    {
      uint16_t cell = spherical_image.at<uint16_t>(row, col);
      if(cell == 0)
      {
        continue;
      }
      // convert the spherical image pixel to a point cloud point
      double rho = static_cast<double>(cell) / 1000.0; // meters
      double phi = (row - phi_bins / 2.0) / phi_resolution;
      double theta = (col - theta_bins / 2.0) / theta_resolution;
      pcl_itr[0] = rho * std::sin(phi) * std::cos(theta);
      pcl_itr[1] = rho * std::sin(phi) * std::sin(theta);
      pcl_itr[2] = rho * std::cos(phi);
      ++pcl_itr;
      valid_points++;
    }
  }
  modifier.resize(valid_points);

  result->width = valid_points;
  result->height = 1;
  result->row_step = msg.row_step;
  result->point_step = msg.point_step;
  result->is_bigendian = msg.is_bigendian;
  result->is_dense = msg.is_dense;
  result->header = msg.header;
  result->fields = msg.fields;
}


}  // namespace projected_point_cloud_transport
