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
    : public point_cloud_transport::SimpleSubscriberPlugin<draco_point_cloud_transport::CompressedPointCloud2>
{
public:
  std::string getTransportName() const override
  {
    return "draco";
  }

  void shutdown() override;

protected:
  // Overridden to set up reconfigure server
  void subscribeImpl(ros::NodeHandle& nh, const std::string& base_topic, uint32_t queue_size,
                     const Callback& callback, const ros::VoidPtr& tracked_object,
                     const point_cloud_transport::TransportHints& transport_hints) override;

  void internalCallback(const draco_point_cloud_transport::CompressedPointCloud2ConstPtr& message,
                        const Callback& user_cb) override;

  typedef draco_point_cloud_transport::DracoSubscriberConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;
  Config config_;

  void configCb(Config& config, uint32_t level);
};

}
