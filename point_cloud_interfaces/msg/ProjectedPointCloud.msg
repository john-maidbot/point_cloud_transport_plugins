# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: John D'Angelo, john-maidbot

# pinhole projection means that compressed_data is like a depth image with png compression
uint8 PINHOLE_PROJECTION = 0

# spherical projection means that compressed_data is like a 3D lidar scan with png compression
uint8 SPHERICAL_PROJECTION = 1

uint8 projection_type

std_msgs/Header header

# if the pointcloud was originally organized, the user probably wants it to stay that way
bool was_dense

uint8[] compressed_data

# This is the point/orientation that acts as a reference for the pointcloud compression
geometry_msgs/Pose view_point;

# TODO: How to maintain fields data?
# sensor_msgs/PointField[] fields
