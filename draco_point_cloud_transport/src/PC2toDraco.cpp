// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague .. 2019, paplhjak

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <draco/attributes/geometry_attribute.h>
#include <draco/point_cloud/point_cloud.h>
#include <draco/point_cloud/point_cloud_builder.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>

#include <draco_point_cloud_transport/PC2toDraco.h>

namespace draco_point_cloud_transport
{

cras::expected<std::unique_ptr<draco::PointCloud>, std::string> convertPC2toDraco(
    sensor_msgs::PointCloud2 PC2, const std::string& topic, bool deduplicate, bool expert_encoding)
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
      if (ros::param::getCached(topic + "/draco/attribute_mapping/attribute_type/" + field.name,
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
          ros::param::getCached(topic + "/draco/attribute_mapping/rgba_tweak/" + field.name, rgba_tweak);
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
          ROS_ERROR_STREAM("Attribute data type not recognized for " + field.name
                               + " field entry. Using regular type recognition instead.");
          expert_settings_ok = false;
        }
      }
      else
      {
        ROS_ERROR_STREAM("Attribute data type not specified for " + field.name
                             + " field entry. Using regular type recognition instead.");
        ROS_INFO_STREAM("To set attribute type for " + field.name + " field entry, set " + topic
                            + "/draco/attribute_mapping/attribute_type/" + field.name);
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
        attribute_type = it->second;
    }

    // attribute data type switch
    switch (field.datatype)
    {
      case sensor_msgs::PointField::INT8:
        attribute_data_type = draco::DT_INT8;
        break;
      case sensor_msgs::PointField::UINT8:
        attribute_data_type = draco::DT_UINT8;
        break;
      case sensor_msgs::PointField::INT16:
        attribute_data_type = draco::DT_INT16;
        break;
      case sensor_msgs::PointField::UINT16:
        attribute_data_type = draco::DT_UINT16;
        break;
      case sensor_msgs::PointField::INT32:
        attribute_data_type = draco::DT_INT32;
        rgba_tweak_64bit = false;
        break;
      case sensor_msgs::PointField::UINT32:
        attribute_data_type = draco::DT_UINT32;
        rgba_tweak_64bit = false;
        break;
      case sensor_msgs::PointField::FLOAT32:
        attribute_data_type = draco::DT_FLOAT32;
        rgba_tweak_64bit = false;
        break;
      case sensor_msgs::PointField::FLOAT64:
        attribute_data_type = draco::DT_FLOAT64;
        rgba_tweak_64bit = true;
        break;
      default:
        return cras::make_unexpected("Invalid data type in PointCloud2 to Draco conversion");
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
    return cras::make_unexpected("Conversion from sensor_msgs::PointCloud2 to Draco::PointCloud failed");

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
    return cras::make_unexpected("Number of points in Draco::PointCloud differs from sensor_msgs::PointCloud2!");

  return pc;
}

}
