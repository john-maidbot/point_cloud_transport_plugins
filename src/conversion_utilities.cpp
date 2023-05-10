// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague .. 2019, paplhjak

#include <draco_point_cloud_transport/conversion_utilities.h>

namespace draco_point_cloud_transport
{

void assign_description_of_PointCloud2(sensor_msgs::PointCloud2& target,
                                       const draco_point_cloud_transport::CompressedPointCloud2& source)
{
  copyCloudMetadata(target, source);
}

void assign_description_of_PointCloud2(draco_point_cloud_transport::CompressedPointCloud2& target,
                                       const sensor_msgs::PointCloud2& source)
{
  copyCloudMetadata(target, source);
}

void assign_description_of_PointCloud2(sensor_msgs::PointCloud2& target,
                                       const draco_point_cloud_transport::CompressedPointCloud2ConstPtr& source)
{
  copyCloudMetadata(target, *source);
}

}
