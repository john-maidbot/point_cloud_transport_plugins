// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague .. 2019, paplhjak

#pragma once

#include <string>

#include <point_cloud_transport/simple_publisher_plugin.h>
#include <sensor_msgs/PointCloud2.h>

#include <draco_point_cloud_transport/CompressedPointCloud2.h>
#include <draco_point_cloud_transport/DracoPublisherConfig.h>

namespace draco_point_cloud_transport
{

class DracoPublisher
    : public point_cloud_transport::SimplePublisherPlugin<CompressedPointCloud2, DracoPublisherConfig>
{
public:
  std::string getTransportName() const override;

  TypedEncodeResult encodeTyped(const sensor_msgs::PointCloud2& raw,
                                const draco_point_cloud_transport::DracoPublisherConfig& config) const override;

  static void registerPositionField(const std::string& field);
  static void registerColorField(const std::string& field);
  static void registerNormalField(const std::string& field);
};

}
