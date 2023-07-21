/*
 * Copyright (c) 2023, Open Source Robotics Foundation, Inc.
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

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <zlib_point_cloud_transport/zlib_publisher.hpp>

#include "gzip_cpp.hpp"

namespace zlib_point_cloud_transport
{

void ZlibPublisher::declareParameters(const std::string & base_topic)
{
}

std::string ZlibPublisher::getTransportName() const
{
  return "zlib";
}

ZlibPublisher::TypedEncodeResult ZlibPublisher::encodeTyped(
  const sensor_msgs::msg::PointCloud2 & raw) const
{
  gzip::Comp comp(gzip::Comp::Level::Level_7, true);
  auto g_compressed_data =
    comp.Process(&raw.data[0], raw.data.size(), true);

  point_cloud_interfaces::msg::CompressedPointCloud2 compressed;

  int total_size = 0;
  for (const auto & data : g_compressed_data) {
    total_size += static_cast<int>(data->size);
  }

  compressed.compressed_data.resize(total_size);

  int index = 0;
  for (const auto & data : g_compressed_data) {
    memcpy(&compressed.compressed_data[index], data->ptr, data->size);
    index += static_cast<int>(data->size);
  }

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

}  // namespace zlib_point_cloud_transport
