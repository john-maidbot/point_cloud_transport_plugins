// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague .. 2019, paplhjak

#include <memory>
#include <vector>

#include <compression/decode.h>

#include <cras_cpp_common/expected.hpp>
#include <cras_cpp_common/string_utils.hpp>
#include <sensor_msgs/PointCloud2.h>

#include <draco_point_cloud_transport/CompressedPointCloud2.h>
#include <draco_point_cloud_transport/draco_subscriber.h>
#include <draco_point_cloud_transport/DracoToPC2.h>

namespace draco_point_cloud_transport
{

DracoSubscriber::DecodeResult DracoSubscriber::decodeTyped(
    const CompressedPointCloud2& compressed, const DracoSubscriberConfig& config) const
{
  // get size of buffer with compressed data in Bytes
  uint32_t compressed_data_size = compressed.compressed_data.size();

  // empty buffer
  if (compressed_data_size == 0)
    return cras::make_unexpected("Received compressed Draco message with zero length.");

  draco::DecoderBuffer decode_buffer;
  std::vector<unsigned char> vec_data = compressed.compressed_data;

  // Sets the buffer's internal data. Note that no copy of the input data is
  // made so the data owner needs to keep the data valid and unchanged for
  // runtime of the decoder.
  decode_buffer.Init(reinterpret_cast<const char*>(&vec_data[0]), compressed_data_size);

  // create decoder object
  draco::Decoder decoder;
  // set decoder from dynamic reconfiguration
  if (config_.SkipDequantizationPOSITION)
  {
    decoder.SetSkipAttributeTransform(draco::GeometryAttribute::POSITION);
  }
  if (config_.SkipDequantizationNORMAL)
  {
    decoder.SetSkipAttributeTransform(draco::GeometryAttribute::NORMAL);
  }
  if (config_.SkipDequantizationCOLOR)
  {
    decoder.SetSkipAttributeTransform(draco::GeometryAttribute::COLOR);
  }
  if (config_.SkipDequantizationTEX_COORD)
  {
    decoder.SetSkipAttributeTransform(draco::GeometryAttribute::TEX_COORD);
  }
  if (config_.SkipDequantizationGENERIC)
  {
    decoder.SetSkipAttributeTransform(draco::GeometryAttribute::GENERIC);
  }

  // decode buffer into draco point cloud
  const auto res = decoder.DecodePointCloudFromBuffer(&decode_buffer);
  if (!res.ok())
    return cras::make_unexpected(
        cras::format("Draco decoder returned code %i: %s.", res.status().code(), res.status().error_msg()));

  const std::unique_ptr<draco::PointCloud>& decoded_pc = res.value();

  sensor_msgs::PointCloud2Ptr message(new sensor_msgs::PointCloud2);
  const auto convertRes = convertDracoToPC2(*decoded_pc, compressed, *message);
  if (!convertRes)
    return cras::make_unexpected(convertRes.error());

  return message;
}

}
