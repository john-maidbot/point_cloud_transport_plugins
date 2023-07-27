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

  // TODO (john.dangelo@tailos.com): User can configure the projection to be
  // - pin-hole projection relative to an imaginary camera:
  // ---> viewpoint origin, viewpoint direction, and pin-hole camera parameters
  // - spherical projection relative to some origin:
  // ---> viewpoint origin

  // TODO (john.dangelo@tailos.com): Apply selected projection method

  // SPHERICAL PROJECTION
  double phi_resolution = 0.5; // radians
  double theta_resolution = 0.5; // radians
  int phi_bins = 2 * M_PI / phi_resolution;
  int theta_bins = 2 * M_PI / rho_resolution;

  cv::Mat spherical_image{phi_bins, theta_bins, CV_16UC1, cv::Scalar(0)};

  // iterate over the pointcloud and convert to polar coordinates
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(raw, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(raw, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(raw, "z");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    const double& x = *iter_x;
    const double& y = *iter_y;
    const double& z = *iter_z;

    const double rho = std::sqrt(x * x + y * y + z * z);
    const double phi = std::atan2(y, x);
    const double theta = std::acos(z / rho);

    const int phi_index = phi / phi_resolution;
    const int theta_index = theta / theta_resolution;

    auto& cell = spherical_image.at<uint16_t>(phi_index, rho_index);
    cell = std::min(cell, static_cast<uint16_t>(rho * 1000)); // mm resolution
  }

  // TODO (john.dangelo@tailos.com): Apply png compression
  point_cloud_interfaces::msg::ProjectedPointCloud compressed;

  cv::imencode(".png", spherical_image, compressed.compressed_data);

  compressed.width = raw.width;
  compressed.height = raw.height;
  compressed.row_step = raw.row_step;
  compressed.point_step = raw.point_step;
  compressed.is_bigendian = raw.is_bigendian;
  compressed.is_dense = true;
  compressed.header = raw.header;
  compressed.fields = raw.fields;

  return compressed;
}

}  // namespace projected_point_cloud_transport
