// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague .. 2019, paplhjak

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <draco/attributes/geometry_attribute.h>
#include <draco/attributes/geometry_indices.h>
#include <draco/core/draco_types.h>
#include <draco/point_cloud/point_cloud.h>
#include <draco/point_cloud/point_cloud_builder.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>

#include <draco_point_cloud_transport/PC2toDraco.h>

//! Constructor
PC2toDraco::PC2toDraco(sensor_msgs::PointCloud2 PC2, std::string topic)
{
  PC2_ = PC2;
  base_topic_ = topic;

  Initialize();
}

//! Method for converting into Draco pointcloud using draco::PointCloudBuilder
std::unique_ptr<draco::PointCloud> PC2toDraco::convert(bool deduplicate_flag, bool expert_encoding_flag)
{
  // object for conversion into Draco Point Cloud format
  draco::PointCloudBuilder builder;
  // number of points in point cloud
  uint64_t number_of_points = PC2_.height * PC2_.width;
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
  for (const auto& field : PC2_.fields)
  {
    if (expert_encoding_flag)  // find attribute type in user specified parameters
    {
      rgba_tweak = false;
      if (ros::param::getCached(base_topic_ + "/draco/attribute_mapping/attribute_type/" + field.name,
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
          ros::param::getCached(base_topic_ + "/draco/attribute_mapping/rgba_tweak/" + field.name, rgba_tweak);
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
        ROS_INFO_STREAM("To set attribute type for " + field.name + " field entry, set " + base_topic_
                            + "/draco/attribute_mapping/attribute_type/" + field.name);
        expert_settings_ok = false;
      }
    }

    if ((!expert_encoding_flag) || (!expert_settings_ok))  // find attribute type in recognized names
    {
      rgba_tweak = false;
      // attribute type switch
      // TODO: add texture coordinate (TEX_COORD) recognized names
      switch (s_mapStringValues[field.name])
      {
        case enumval1 :  // "x"
        case enumval2 :  // "y"
        case enumval3 :  // "z"
        case enumval4 :  // "pos"
        case enumval5 :  // "position"
          attribute_type = draco::GeometryAttribute::POSITION;
          break;
        case enumval10 :  // "rgb"
        case enumval11 :  // "rgba"
          //
          rgba_tweak = true;
        case enumval6 :  // "r"
        case enumval7 :  // "g"
        case enumval8 :  // "b"
        case enumval9 :  // "a"
          attribute_type = draco::GeometryAttribute::COLOR;
          break;
        case enumval12 :  // "nx"
        case enumval13 :  // "ny"
        case enumval14 :  // "nz"
          attribute_type = draco::GeometryAttribute::NORMAL;
          break;
        case enumvalGeneric :  // all unrecognized attributes
          attribute_type = draco::GeometryAttribute::GENERIC;
        default:ROS_ASSERT("Unknown enum type.");
          break;
      }  // attribute type switch end
    }  // find attribute type in recognized names end

    // attribute data type switch
    switch (field.datatype)
    {
      case 1 :attribute_data_type = draco::DT_INT8;
        break;
      case 2 :attribute_data_type = draco::DT_UINT8;
        break;
      case 3 :attribute_data_type = draco::DT_INT16;
        break;
      case 4 :attribute_data_type = draco::DT_UINT16;
        break;
      case 5 :attribute_data_type = draco::DT_INT32;
        rgba_tweak_64bit = false;
        break;
      case 6 :attribute_data_type = draco::DT_UINT32;
        rgba_tweak_64bit = false;
        break;
      case 7 :attribute_data_type = draco::DT_FLOAT32;
        rgba_tweak_64bit = false;
        break;
      case 8 :attribute_data_type = draco::DT_FLOAT64;
        rgba_tweak_64bit = true;
        break;
      default:attribute_data_type = draco::DT_INVALID;
        // RAISE ERROR - INVALID DATA TYPE
        ROS_FATAL_STREAM(" Invalid data type in PointCloud2 to Draco conversion");
        break;
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
          static_cast<int>(att_ids.back()), &PC2_.data[0] + field.offset, PC2_.point_step);
    }
  }
  // finalize point cloud *** builder.Finalize(bool deduplicate) ***
  std::unique_ptr<draco::PointCloud> pc = builder.Finalize(deduplicate_flag);

  if (pc == nullptr)
  {
    ROS_FATAL_STREAM("Conversion from sensor_msgs::PointCloud2 to Draco::PointCloud failed");
  }

  // add metadata to point cloud
  std::unique_ptr<draco::GeometryMetadata> metadata = std::make_unique<draco::GeometryMetadata>();

  if (deduplicate_flag)
  {
    metadata->AddEntryInt("deduplicate", 1);  // deduplication=true flag
  }
  else
  {
    metadata->AddEntryInt("deduplicate", 0);  // deduplication=false flag
  }
  pc->AddMetadata(std::move(metadata));

  if ((pc->num_points() != number_of_points) && !deduplicate_flag)
  {
    ROS_FATAL_STREAM("Number of points in Draco::PointCloud differs from sensor_msgs::PointCloud2!");
  }
  return pc;
}
