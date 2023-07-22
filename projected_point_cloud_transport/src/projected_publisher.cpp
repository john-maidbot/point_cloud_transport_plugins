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
  // - assume organized point cloud:
  // ---> assumes the pointcloud is already organized like an image and can be compressed right away
  // - pin-hole projection relative to an imaginary camera:
  // ---> viewpoint origin, viewpoint direction, and pin-hole camera parameters
  // - spherical projection relative to some origin:
  // ---> viewpoint origin

  // TODO (john.dangelo@tailos.com): Apply selected projection method

  // TODO (john.dangelo@tailos.com): Apply png compression

  compressed.width = raw.width;
  compressed.height = raw.height;
  compressed.row_step = raw.row_step;
  compressed.point_step = raw.point_step;
  compressed.is_bigendian = raw.is_bigendian;
  compressed.is_dense = raw.is_dense;
  compressed.header = raw.header;
  compressed.fields = raw.fields;

  return compressed;
}

}  // namespace projected_point_cloud_transport
