// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague .. 2019, paplhjak

#pragma once

#include <sensor_msgs/PointCloud2.h>

#include <draco_point_cloud_transport/CompressedPointCloud2.h>


namespace draco_point_cloud_transport
{

template<typename PC1, typename PC2>
void copyCloudMetadata(PC1& target, const PC2& source)
{
  target.header = source.header;
  target.height = source.height;
  target.width = source.width;
  target.fields = source.fields;
  target.is_bigendian = source.is_bigendian;
  target.point_step = source.point_step;
  target.row_step = source.row_step;
  target.is_dense = source.is_dense;
}

//! assigns header, width, ... from compressed to regular
void assign_description_of_PointCloud2(sensor_msgs::PointCloud2& target,
                                       const draco_point_cloud_transport::CompressedPointCloud2& source);

//! assigns header, width, ... from regular to compressed
void assign_description_of_PointCloud2(draco_point_cloud_transport::CompressedPointCloud2& target,
                                       const sensor_msgs::PointCloud2& source);

//! assigns header, width, ... from compressedConstPtr to regular
void assign_description_of_PointCloud2(sensor_msgs::PointCloud2& target,
                                       const draco_point_cloud_transport::CompressedPointCloud2ConstPtr& source);

}
