// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague .. 2019, paplhjak

#include <memory>
#include <string>
#include <vector>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <point_cloud_interfaces/msg/compressed_point_cloud2.hpp>
#include <point_cloud_transport/expected.hpp>

#include <draco/point_cloud/point_cloud.h>
#include <draco/compression/decode.h>

#include <draco_point_cloud_transport/conversion_utilities.h>
#include <draco_point_cloud_transport/draco_subscriber.hpp>

namespace draco_point_cloud_transport
{

//! Method for converting into sensor_msgs::msg::PointCloud2
cras::expected<bool, std::string> convertDracoToPC2(
    const draco::PointCloud& pc,
    const point_cloud_interfaces::msg::CompressedPointCloud2& compressed_PC2,
    sensor_msgs::msg::PointCloud2& PC2)
{
  // number of all attributes of point cloud
  int32_t number_of_attributes = pc.num_attributes();

  // number of points in pointcloud
  draco::PointIndex::ValueType number_of_points = pc.num_points();

  PC2.data.resize(number_of_points * compressed_PC2.point_step);

  // for each attribute
  for (int32_t att_id = 0; att_id < number_of_attributes; att_id++)
  {
    // get attribute
    const draco::PointAttribute* attribute = pc.attribute(att_id);

    // check if attribute is valid
    if (!attribute->IsValid())
      return cras::make_unexpected("In point_cloud_transport::DracoToPC2, attribute of Draco pointcloud is not valid!");

    // get offset of attribute in data structure
    uint32_t attribute_offset = compressed_PC2.fields[att_id].offset;

    // for each point in point cloud
    for (draco::PointIndex::ValueType point_index = 0; point_index < number_of_points; point_index++)
    {
      // get pointer to corresponding memory in PointCloud2 data
      uint8_t* out_data = &PC2.data[static_cast<int>(compressed_PC2.point_step * point_index + attribute_offset)];

      // read value from Draco pointcloud to out_data
      attribute->GetValue(draco::AttributeValueIndex(point_index), out_data);
    }
  }

  // copy PointCloud2 description (header, width, ...)
  copyCloudMetadata(PC2, compressed_PC2);

  return true;
}

std::string DracoSubscriber::getTransportName() const
{
  return "draco";
}

DracoSubscriber::DecodeResult DracoSubscriber::decodeTyped(
    const point_cloud_interfaces::msg::CompressedPointCloud2& compressed) const
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
        "Draco decoder returned code "+std::to_string(res.status().code())+": "+res.status().error_msg()+".");

  const std::unique_ptr<draco::PointCloud>& decoded_pc = res.value();

  auto message = std::make_shared<sensor_msgs::msg::PointCloud2>();
  const auto convertRes = convertDracoToPC2(*decoded_pc, compressed, *message);
  if (!convertRes)
    return cras::make_unexpected(convertRes.error());

  return message;
}

}
