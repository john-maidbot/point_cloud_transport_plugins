// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague .. 2019, paplhjak

#include <pluginlib/class_list_macros.h>

#include <point_cloud_transport/publisher_plugin.h>
#include <point_cloud_transport/subscriber_plugin.h>

#include <draco_point_cloud_transport/draco_publisher.h>
#include <draco_point_cloud_transport/draco_subscriber.h>

PLUGINLIB_EXPORT_CLASS(draco_point_cloud_transport::DracoPublisher, point_cloud_transport::PublisherPlugin)
PLUGINLIB_EXPORT_CLASS(draco_point_cloud_transport::DracoSubscriber, point_cloud_transport::SubscriberPlugin)
