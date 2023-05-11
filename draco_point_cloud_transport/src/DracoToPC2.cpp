// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague .. 2019, paplhjak

#include <string>

#include <attributes/geometry_indices.h>
#include <point_cloud/point_cloud.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <draco_point_cloud_transport/CompressedPointCloud2.h>
#include <draco_point_cloud_transport/conversion_utilities.h>
#include <draco_point_cloud_transport/DracoToPC2.h>

namespace draco_point_cloud_transport
{

//! Method for converting into sensor_msgs::PointCloud2
cras::expected<bool, std::string> convertDracoToPC2(
    const draco::PointCloud& pc,
    const draco_point_cloud_transport::CompressedPointCloud2& compressed_PC2,
    sensor_msgs::PointCloud2& PC2)
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

  // if points were deduplicated, overwrite height and width
  int deduplicate;
  pc.GetMetadata()->GetEntryInt("deduplicate", &deduplicate);

  if (deduplicate == 1)
  {
    PC2.width = number_of_points;
    PC2.height = 1;
    PC2.is_dense = false;
  }

  return true;
}

}
