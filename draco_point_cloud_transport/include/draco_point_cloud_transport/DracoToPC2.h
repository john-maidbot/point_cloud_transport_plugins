// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague .. 2019, paplhjak

#pragma once

#include <string>

#include <point_cloud/point_cloud.h>

#include <cras_cpp_common/expected.hpp>
#include <sensor_msgs/PointCloud2.h>

#include <draco_point_cloud_transport/CompressedPointCloud2.h>

namespace draco_point_cloud_transport
{

//! Method for converting into sensor_msgs::PointCloud2
cras::expected<bool, std::string> convertDracoToPC2(
    const draco::PointCloud& pc,
    const draco_point_cloud_transport::CompressedPointCloud2& compressed_PC2,
    sensor_msgs::PointCloud2& PC2);

}
