// Copyright 2023 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "pcl_point_cloud_transport/pcl_sensor_msgs_pointcloud2_type_adapter.hpp"

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(
  pcl_point_cloud_transport::PCLContainer,
  sensor_msgs::msg::PointCloud2);

class ShowPointCloud : public rclcpp::Node
{
public:
  ShowPointCloud()
  : Node("showpointcloud")
  {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    initialize();
  }

private:
  void initialize()
  {
    // Set quality of service profile based on command line options.
    auto qos = rclcpp::QoS(
      rclcpp::QoSInitialization(
        // The history policy determines how messages are saved until taken by
        // the reader.
        // KEEP_ALL saves all messages until they are taken.
        // KEEP_LAST enforces a limit on the number of messages that are saved,
        // specified by the "depth" parameter.
        history_policy_,
        // Depth represents how many messages to store in history when the
        // history policy is KEEP_LAST.
        depth_
    ));
    // The reliability policy can be reliable, meaning that the underlying transport layer will try
    // ensure that every message gets received in order, or best effort, meaning that the transport
    // makes no guarantees about the order or reliability of delivery.
    qos.reliability(reliability_policy_);
    auto callback =
      [this](const pcl_point_cloud_transport::PCLContainer & container) {
        process_pointcloud(container, this->get_logger());
      };

    RCLCPP_INFO(this->get_logger(), "Subscribing to topic '/pct/point_cloud/pcl'");
    sub_ = create_subscription<pcl_point_cloud_transport::PCLContainer>(
      "/pct/point_cloud/pcl",
      rclcpp::SensorDataQoS(), callback);
  }

  /// Convert the ROS Image message to an OpenCV matrix and display it to the user.
  // \param[in] container The image message to show.
  void process_pointcloud(
    const pcl_point_cloud_transport::PCLContainer & container, rclcpp::Logger logger)
  {
    RCLCPP_INFO(logger, "Received point_cloud #%s", container.header().frame_id.c_str());
  }

  rclcpp::Subscription<pcl_point_cloud_transport::PCLContainer>::SharedPtr sub_;
  size_t depth_ = rmw_qos_profile_default.depth;
  rmw_qos_reliability_policy_t reliability_policy_ = rmw_qos_profile_default.reliability;
  rmw_qos_history_policy_t history_policy_ = rmw_qos_profile_default.history;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ShowPointCloud>());
  rclcpp::shutdown();
  return 0;
}
