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
  const point_cloud_interfaces::msg::CompressedPointCloud2 & msg) const
{
  auto result = std::make_shared<sensor_msgs::msg::PointCloud2>();

  // TODO (john.dangelo@tailos.com): Apply png decompression

  // TODO (john.dangelo@tailos.com): User can configure the deprojection to be
  // - assume organized point cloud:
  // ---> assumes the pointcloud is already organized like an image and can be compressed right away
  // - pin-hole deprojection relative to an imaginary camera:
  // ---> viewpoint origin, viewpoint direction, and pin-hole camera parameters
  // - spherical deprojection relative to some origin:
  // ---> viewpoint origin

  // TODO (john.dangelo@tailos.com): Apply selected deprojection method

  result->width = msg.width;
  result->height = msg.height;
  result->row_step = msg.row_step;
  result->point_step = msg.point_step;
  result->is_bigendian = msg.is_bigendian;
  result->is_dense = msg.is_dense;
  result->header = msg.header;
  result->fields = msg.fields;

  return result;
}

}  // namespace projected_point_cloud_transport
