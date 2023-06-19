// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague .. 2019, paplhjak

#pragma once

#include <string>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <point_cloud_transport/simple_publisher_plugin.hpp>

#include <point_cloud_interfaces/msg/compressed_point_cloud2.hpp>

namespace draco_point_cloud_transport
{

class DracoPublisher
    : public point_cloud_transport::SimplePublisherPlugin<point_cloud_interfaces::msg::CompressedPointCloud2, DracoPublisherConfig>
{
public:
  std::string getTransportName() const override;

  TypedEncodeResult encodeTyped(const sensor_msgs::msg::PointCloud2& raw,
                                const draco_point_cloud_transport::DracoPublisherConfig& config) const override;

  static void registerPositionField(const std::string& field);
  static void registerColorField(const std::string& field);
  static void registerNormalField(const std::string& field);
};

}
