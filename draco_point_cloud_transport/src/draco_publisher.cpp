// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague .. 2019, paplhjak

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <draco/compression/expert_encode.h>
#include <draco/compression/encode.h>
#include <draco/point_cloud/point_cloud_builder.h>

#include <draco_point_cloud_transport/cloud.hpp>
#include <draco_point_cloud_transport/expected.hpp>
#include <draco_point_cloud_transport/msg/compressed_point_cloud2.hpp>
#include <draco_point_cloud_transport/conversion_utilities.h>
#include <draco_point_cloud_transport/draco_publisher.h>

namespace draco_point_cloud_transport
{

static std::unordered_map<std::string, draco::GeometryAttribute::Type> attributeTypes = {
    {"x", draco::GeometryAttribute::Type::POSITION},
    {"y", draco::GeometryAttribute::Type::POSITION},
    {"z", draco::GeometryAttribute::Type::POSITION},
    {"pos", draco::GeometryAttribute::Type::POSITION},
    {"position", draco::GeometryAttribute::Type::POSITION},
    {"vp_x", draco::GeometryAttribute::Type::POSITION},
    {"vp_y", draco::GeometryAttribute::Type::POSITION},
    {"vp_z", draco::GeometryAttribute::Type::POSITION},
    {"rgb", draco::GeometryAttribute::Type::COLOR},
    {"rgba", draco::GeometryAttribute::Type::COLOR},
    {"r", draco::GeometryAttribute::Type::COLOR},
    {"g", draco::GeometryAttribute::Type::COLOR},
    {"b", draco::GeometryAttribute::Type::COLOR},
    {"a", draco::GeometryAttribute::Type::COLOR},
    {"nx", draco::GeometryAttribute::Type::NORMAL},
    {"ny", draco::GeometryAttribute::Type::NORMAL},
    {"nz", draco::GeometryAttribute::Type::NORMAL},
    {"normal_x", draco::GeometryAttribute::Type::NORMAL},
    {"normal_y", draco::GeometryAttribute::Type::NORMAL},
    {"normal_z", draco::GeometryAttribute::Type::NORMAL},
};

cras::expected<std::unique_ptr<draco::PointCloud>, std::string> convertPC2toDraco(
    const sensor_msgs::msg::PointCloud2& PC2, const std::string& topic, bool deduplicate, bool expert_encoding)
{
  // object for conversion into Draco Point Cloud format
  draco::PointCloudBuilder builder;
  // number of points in point cloud
  uint64_t number_of_points = PC2.height * PC2.width;
  // initialize builder object, requires prior knowledge of point cloud size for buffer allocation
  builder.Start(number_of_points);
  // vector to hold IDs of attributes for builder object
  std::vector<int> att_ids;

  // initialize to invalid
  draco::GeometryAttribute::Type attribute_type = draco::GeometryAttribute::INVALID;
  draco::DataType attribute_data_type = draco::DT_INVALID;

  // tracks if rgb/rgba was encountered (4 colors saved as one variable,
  bool rgba_tweak = false;
  bool rgba_tweak_64bit = false;

  // tracks if all necessary parameters were set for expert encoder
  bool expert_settings_ok = true;

  std::string expert_attribute_data_type;

  // fill in att_ids with attributes from PointField[] fields
  for (const auto& field : PC2.fields)
  {
    if (expert_encoding)  // find attribute type in user specified parameters
    {
      rgba_tweak = false;

      if (node_->get_parameter(topic + "/draco/attribute_mapping/attribute_type/" + field.name,
                                expert_attribute_data_type))
      {
        if (expert_attribute_data_type == "POSITION")  // if data type is POSITION
        {
          attribute_type = draco::GeometryAttribute::POSITION;
        }
        else if (expert_attribute_data_type == "NORMAL")  // if data type is NORMAL
        {
          attribute_type = draco::GeometryAttribute::NORMAL;
        }
        else if (expert_attribute_data_type == "COLOR")  // if data type is COLOR
        {
          attribute_type = draco::GeometryAttribute::COLOR;
          node_->get_parameter(topic + "/draco/attribute_mapping/rgba_tweak/" + field.name, rgba_tweak);
        }
        else if (expert_attribute_data_type == "TEX_COORD")  // if data type is TEX_COORD
        {
          attribute_type = draco::GeometryAttribute::TEX_COORD;
        }
        else if (expert_attribute_data_type == "GENERIC")  // if data type is GENERIC
        {
          attribute_type = draco::GeometryAttribute::GENERIC;
        }
        else
        {
          CRAS_ERROR_STREAM("Attribute data type not recognized for " + field.name + " field entry. "
                            "Using regular type recognition instead.");
          expert_settings_ok = false;
        }
      }
      else
      {
        CRAS_ERROR_STREAM("Attribute data type not specified for " + field.name + " field entry."
                          "Using regular type recognition instead.");
        CRAS_INFO_STREAM("To set attribute type for " + field.name + " field entry, set " + topic +
                         "/draco/attribute_mapping/attribute_type/" + field.name);
        expert_settings_ok = false;
      }
    }

    // find attribute type in recognized names
    if ((!expert_encoding) || (!expert_settings_ok))
    {
      rgba_tweak = field.name == "rgb" || field.name == "rgba";
      attribute_type = draco::GeometryAttribute::GENERIC;
      const auto& it = attributeTypes.find(field.name);
      if (it != attributeTypes.end())
      {
        attribute_type = it->second;
      }
    }

    // attribute data type switch
    switch (field.datatype)
    {
      case sensor_msgs::PointField::INT8:attribute_data_type = draco::DT_INT8;
        break;
      case sensor_msgs::PointField::UINT8:attribute_data_type = draco::DT_UINT8;
        break;
      case sensor_msgs::PointField::INT16:attribute_data_type = draco::DT_INT16;
        break;
      case sensor_msgs::PointField::UINT16:attribute_data_type = draco::DT_UINT16;
        break;
      case sensor_msgs::PointField::INT32:attribute_data_type = draco::DT_INT32;
        rgba_tweak_64bit = false;
        break;
      case sensor_msgs::PointField::UINT32:attribute_data_type = draco::DT_UINT32;
        rgba_tweak_64bit = false;
        break;
      case sensor_msgs::PointField::FLOAT32:attribute_data_type = draco::DT_FLOAT32;
        rgba_tweak_64bit = false;
        break;
      case sensor_msgs::PointField::FLOAT64:attribute_data_type = draco::DT_FLOAT64;
        rgba_tweak_64bit = true;
        break;
      default:return cras::make_unexpected("Invalid data type in PointCloud2 to Draco conversion");
    }  // attribute data type switch end

    // add attribute to point cloud builder
    if (rgba_tweak)  // attribute is rgb/rgba color
    {
      if (rgba_tweak_64bit)  // attribute data type is 64bits long, each color is encoded in 16bits
      {
        att_ids.push_back(builder.AddAttribute(attribute_type, 4 * field.count, draco::DT_UINT16));
      }
      else  // attribute data type is 32bits long, each color is encoded in 8bits
      {
        att_ids.push_back(builder.AddAttribute(attribute_type, 4 * field.count, draco::DT_UINT8));
      }
    }
    else  // attribute is not rgb/rgba color, this is the default behavior
    {
      att_ids.push_back(builder.AddAttribute(attribute_type, field.count, attribute_data_type));
    }
    // Set attribute values for the last added attribute
    if ((!att_ids.empty()) && (attribute_data_type != draco::DT_INVALID))
    {
      builder.SetAttributeValuesForAllPoints(
          static_cast<int>(att_ids.back()), &PC2.data[0] + field.offset, PC2.point_step);
    }
  }
  // finalize point cloud *** builder.Finalize(bool deduplicate) ***
  std::unique_ptr<draco::PointCloud> pc = builder.Finalize(deduplicate);

  if (pc == nullptr)
  {
    return cras::make_unexpected("Conversion from sensor_msgs::msg::PointCloud2 to Draco::PointCloud failed");
  }

  // add metadata to point cloud
  std::unique_ptr<draco::GeometryMetadata> metadata = std::make_unique<draco::GeometryMetadata>();

  if (deduplicate)
  {
    metadata->AddEntryInt("deduplicate", 1);  // deduplication=true flag
  }
  else
  {
    metadata->AddEntryInt("deduplicate", 0);  // deduplication=false flag
  }
  pc->AddMetadata(std::move(metadata));

  if ((pc->num_points() != number_of_points) && !deduplicate)
  {
    return cras::make_unexpected("Number of points in Draco::PointCloud differs from sensor_msgs::msg::PointCloud2!");
  }

  return std::move(pc);  // std::move() has to be here for GCC 7
}

std::string DracoPublisher::getTransportName() const
{
  return "draco";
}

DracoPublisher::TypedEncodeResult DracoPublisher::encodeTyped(
    const sensor_msgs::msg::PointCloud2& raw, const draco_point_cloud_transport::DracoPublisherConfig& config) const
{
  // Remove invalid points if the cloud contains them - draco cannot cope with them
  sensor_msgs::msg::PointCloud2::SharedPtr rawCleaned;
  if (!raw.is_dense) {
    rawCleaned = std::make_shared<sensor_msgs::msg::PointCloud2>();
    CREATE_FILTERED_CLOUD(raw, *rawCleaned, false, std::isfinite(*x_it) && std::isfinite(*y_it) && std::isfinite(*z_it))
  }

  const sensor_msgs::msg::PointCloud2& rawDense = raw.is_dense ? raw : *rawCleaned;

  // Compressed message
  draco_point_cloud_transport::msg::CompressedPointCloud2 compressed;

  copyCloudMetadata(compressed, rawDense);

  auto res = convertPC2toDraco(rawDense, base_topic_, config.deduplicate, config.expert_attribute_types);
  if (!res)
  {
    return cras::make_unexpected(res.error());
  }

  const auto& pc = res.value();

  if (config.deduplicate)
  {
      compressed.height = 1;
      compressed.width = pc->num_points();
      compressed.row_step = compressed.width * compressed.point_step;
      compressed.is_dense = true;
  }

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

        for (const auto& field : rawDense.fields)
        {
          if (node_->get_parameter(base_topic_ + "/draco/attribute_mapping/quantization_bits/" + field.name,
                                    attribute_quantization_bits))
          {
            expert_encoder.SetAttributeQuantization(att_id, attribute_quantization_bits);
          }
          else
          {
            RCLCPP_ERROR_STREAM(node_->get_logger(), "Attribute quantization not specified for " + field.name + " field entry. "
                              "Using regular encoder instead.");
            RCLCPP_INFO_STREAM(node_->get_logger(), "To set quantization for " + field.name + " field entry, set " + base_topic_ +
                             "/draco/attribute_mapping/quantization_bits/" + field.name);
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
      // TODO: Fix with proper format
      return cras::make_unexpected(
          "Draco encoder returned code "+std::to_string(status.code())+": "+status.error_msg()+".");
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
    {
      return cras::make_unexpected(
          "Draco encoder returned code "+std::to_string(status.code())+": "+status.error_msg()+".");
    }
  }
  // regular encoder end

  uint32_t compressed_data_size = encode_buffer.size();
  auto cast_buffer = reinterpret_cast<const unsigned char*>(encode_buffer.data());
  std::vector<unsigned char> vec_data(cast_buffer, cast_buffer + compressed_data_size);
  compressed.compressed_data = vec_data;

  return compressed;
}

void DracoPublisher::registerPositionField(const std::string& field)
{
  attributeTypes[field] = draco::GeometryAttribute::Type::POSITION;
}

void DracoPublisher::registerColorField(const std::string& field)
{
  attributeTypes[field] = draco::GeometryAttribute::Type::COLOR;
}

void DracoPublisher::registerNormalField(const std::string& field)
{
  attributeTypes[field] = draco::GeometryAttribute::Type::NORMAL;
}

}
