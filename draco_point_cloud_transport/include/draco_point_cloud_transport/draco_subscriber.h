// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague .. 2019, paplhjak

#pragma once

#include <string>

#include <point_cloud_interfaces/msg/compressed_point_cloud2.hpp>

#include <point_cloud_transport/simple_subscriber_plugin.hpp>
#include <point_cloud_transport/transport_hints.hpp>

namespace draco_point_cloud_transport
{

class DracoSubscriber
    : public point_cloud_transport::SimpleSubscriberPlugin<CompressedPointCloud2, DracoSubscriberConfig>
{
public:
  std::string getTransportName() const override;

  DecodeResult decodeTyped(const CompressedPointCloud2& compressed, const DracoSubscriberConfig& config) const override;
};

}
