// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague .. 2019, paplhjak

#pragma once

#include <memory>

#include <draco/point_cloud/point_cloud.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <draco_point_cloud_transport/conversion_utilities.h>
#include <draco_point_cloud_transport/CompressedPointCloud2.h>

class DracoToPC2
{
public:
  //! Constructor.
  explicit DracoToPC2(std::unique_ptr<draco::PointCloud>&& pc,
                      const draco_point_cloud_transport::CompressedPointCloud2ConstPtr& compressed_PC2);

  //! Method for converting into sensor_msgs::PointCloud2
  sensor_msgs::PointCloud2 convert();

private:
  //! Message to be converted
  std::unique_ptr<draco::PointCloud> pc_;

  //! Structure to hold information about sensor_msgs::PointCloud2
  draco_point_cloud_transport::CompressedPointCloud2ConstPtr compressed_PC2_;
};
