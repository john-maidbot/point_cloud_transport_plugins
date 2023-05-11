// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague .. 2019, paplhjak

#pragma once

#include <string>

#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include <point_cloud_transport/simple_subscriber_plugin.h>
#include <point_cloud_transport/transport_hints.h>

#include <draco_point_cloud_transport/CompressedPointCloud2.h>
#include <draco_point_cloud_transport/DracoSubscriberConfig.h>

namespace draco_point_cloud_transport
{

class DracoSubscriber
    : public point_cloud_transport::SimpleSubscriberPlugin<CompressedPointCloud2, DracoSubscriberConfig>
{
public:
  std::string getTransportName() const override
  {
    return "draco";
  }

  DecodeResult decodeTyped(const CompressedPointCloud2& compressed, const DracoSubscriberConfig& config) const override;
};

}
