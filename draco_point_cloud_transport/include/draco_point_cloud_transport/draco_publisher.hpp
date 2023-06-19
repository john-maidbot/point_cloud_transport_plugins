// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague .. 2019, paplhjak

#pragma once

#include <string>

#include <draco/point_cloud/point_cloud.h>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <point_cloud_transport/simple_publisher_plugin.hpp>

#include <point_cloud_interfaces/msg/compressed_point_cloud2.hpp>

namespace draco_point_cloud_transport
{

class DracoPublisher
    : public point_cloud_transport::SimplePublisherPlugin<point_cloud_interfaces::msg::CompressedPointCloud2>
{
public:
  std::string getTransportName() const override;

  TypedEncodeResult encodeTyped(const sensor_msgs::msg::PointCloud2& raw) const override;

  static void registerPositionField(const std::string& field);
  static void registerColorField(const std::string& field);
  static void registerNormalField(const std::string& field);

private:
  cras::expected<std::unique_ptr<draco::PointCloud>, std::string> convertPC2toDraco(
      const sensor_msgs::msg::PointCloud2& PC2, const std::string& topic, bool deduplicate, bool expert_encoding) const;

  struct DracoPublisherConfig{
    int encode_speed = 7;
    int decode_speed = 7;
    int method_enum = 0;
    int encode_method = 0;
    bool deduplicate = true;
    bool force_quantization = true;
    int quantization_POSITION = 14;
    int quantization_NORMAL = 14;
    int quantization_COLOR = 14;
    int quantization_TEX_COORD = 14;
    int quantization_GENERIC = 14;
    bool expert_quantization = false;
    bool expert_attribute_types = false;
  };

  DracoPublisherConfig config_;
};

}
