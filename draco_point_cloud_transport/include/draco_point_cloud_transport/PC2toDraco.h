// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague .. 2019, paplhjak

#pragma once

#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <cras_cpp_common/expected.hpp>
#include <sensor_msgs/PointCloud2.h>

#include <draco/point_cloud/point_cloud.h>
#include <draco/point_cloud/point_cloud_builder.h>

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

//! Method for converting into Draco pointcloud using draco::PointCloudBuilder
cras::expected<std::unique_ptr<draco::PointCloud>, std::string> convertPC2toDraco(
    sensor_msgs::PointCloud2 PC2, const std::string& topic, bool deduplicate, bool expert_encoding);

}
