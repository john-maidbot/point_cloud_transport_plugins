// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague .. 2019, paplhjak

#include <string>
#include <vector>

#include <compression/expert_encode.h>
#include <compression/encode.h>

#include <cras_cpp_common/expected.hpp>
#include <cras_cpp_common/string_utils.hpp>
#include <ros/ros.h>

#include <draco_point_cloud_transport/conversion_utilities.h>
#include <draco_point_cloud_transport/CompressedPointCloud2.h>
#include <draco_point_cloud_transport/draco_publisher.h>
#include <draco_point_cloud_transport/PC2toDraco.h>

namespace draco_point_cloud_transport
{

std::string DracoPublisher::getTransportName() const
{
  return "draco";
}

DracoPublisher::TypedEncodeResult DracoPublisher::encodeTyped(
    const sensor_msgs::PointCloud2& raw, const draco_point_cloud_transport::DracoPublisherConfig& config) const
{
  // Compressed message
  draco_point_cloud_transport::CompressedPointCloud2 compressed;

  copyCloudMetadata(compressed, raw);

  auto res = convertPC2toDraco(raw, base_topic_, config.deduplicate, config.expert_attribute_types);
  if (!res)
    return cras::make_unexpected(res.error());

  const auto& pc = res.value();
  draco::EncoderBuffer encode_buffer;

  // tracks if all necessary parameters were set for expert encoder
  bool expert_settings_ok = true;

  // expert encoder
  if (config.expert_quantization)
  {
    draco::ExpertEncoder expert_encoder(*pc);
    expert_encoder.SetSpeedOptions(config.encode_speed, config.decode_speed);

    // default
    if ((config.encode_method == 0) && (!config.force_quantization))
    {
      // let draco handle method selection
    }
    // force kd tree
    else if ((config.encode_method == 1) || (config.force_quantization))
    {
      if (config.force_quantization)
      {
        // keep track of which attribute is being processed
        int att_id = 0;
        int attribute_quantization_bits;

        for (const auto& field : raw.fields)
        {
          if (ros::param::getCached(base_topic_ + "/draco/attribute_mapping/quantization_bits/" + field.name,
                                    attribute_quantization_bits))
          {
            expert_encoder.SetAttributeQuantization(att_id, attribute_quantization_bits);
          }
          else
          {
            ROS_ERROR_STREAM("Attribute quantization not specified for " + field.name
                                  + " field entry. Using regular encoder instead.");
            ROS_INFO_STREAM("To set quantization for " + field.name + " field entry, set " + base_topic_
                                 + "/draco/attribute_mapping/quantization_bits/" + field.name);
            expert_settings_ok = false;
          }
          att_id++;
        }
      }
      expert_encoder.SetEncodingMethod(draco::POINT_CLOUD_KD_TREE_ENCODING);
    }
    // force sequential encoding
    else
    {
      expert_encoder.SetEncodingMethod(draco::POINT_CLOUD_SEQUENTIAL_ENCODING);
    }

    // encodes point cloud and raises error if encoding fails
    // draco::Status status = encoder.EncodePointCloudToBuffer(*pc, &encode_buffer);
    draco::Status status = expert_encoder.EncodeToBuffer(&encode_buffer);

    if (status.code() != 0)
    {
      return cras::make_unexpected(
        cras::format("Draco encoder returned code %i: %s.", status.code(), status.error_msg()));
    }
  }
  // expert encoder end

  // regular encoder
  if ((!config.expert_quantization) || (!expert_settings_ok))
  {
    draco::Encoder encoder;
    encoder.SetSpeedOptions(config.encode_speed, config.decode_speed);

    // default
    if ((config.encode_method == 0) && (!config.force_quantization))
    {
      // let draco handle method selection
    }
    // force kd tree
    else if ((config.encode_method == 1) || (config.force_quantization))
    {
      if (config.force_quantization)
      {
        encoder.SetAttributeQuantization(draco::GeometryAttribute::POSITION, config.quantization_POSITION);
        encoder.SetAttributeQuantization(draco::GeometryAttribute::NORMAL, config.quantization_NORMAL);
        encoder.SetAttributeQuantization(draco::GeometryAttribute::COLOR, config.quantization_COLOR);
        encoder.SetAttributeQuantization(draco::GeometryAttribute::TEX_COORD, config.quantization_TEX_COORD);
        encoder.SetAttributeQuantization(draco::GeometryAttribute::GENERIC, config.quantization_GENERIC);
      }
      encoder.SetEncodingMethod(draco::POINT_CLOUD_KD_TREE_ENCODING);
    }
    // force sequential encoding
    else
    {
      encoder.SetEncodingMethod(draco::POINT_CLOUD_SEQUENTIAL_ENCODING);
    }

    // encodes point cloud and raises error if encoding fails
    draco::Status status = encoder.EncodePointCloudToBuffer(*pc, &encode_buffer);

    if (!status.ok())
      return cras::make_unexpected(
        cras::format("Draco encoder returned code %i: %s.", status.code(), status.error_msg()));
  }
  // regular encoder end

  uint32_t compressed_data_size = encode_buffer.size();
  auto cast_buffer = reinterpret_cast<const unsigned char*>(encode_buffer.data());
  std::vector<unsigned char> vec_data(cast_buffer, cast_buffer + compressed_data_size);
  compressed.compressed_data = vec_data;

  return compressed;
}

void DracoPublisher::registerPositionField(const std::string& field) const
{
  attributeTypes[field] = draco::GeometryAttribute::Type::POSITION;
}

void DracoPublisher::registerColorField(const std::string& field) const
{
  attributeTypes[field] = draco::GeometryAttribute::Type::COLOR;
}

void DracoPublisher::registerNormalField(const std::string& field) const
{
  attributeTypes[field] = draco::GeometryAttribute::Type::NORMAL;
}

}
