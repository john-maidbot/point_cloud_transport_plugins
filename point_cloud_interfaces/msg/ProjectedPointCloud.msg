# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: John D'Angelo, john-maidbot

# pinhole projection means that compressed_data is like a depth image with png compression
uint8 PINHOLE_PROJECTION = 0

# spherical projection means that compressed_data is like a 3D lidar scan with png compression
uint8 SPHERICAL_PROJECTION = 1

uint8 projection_type

std_msgs/Header header

uint32 height
uint32 width

# TODO: How to maintain fields data? we would probably need a voxel grid
sensor_msgs/PointField[] fields

bool is_bigendian

uint32 row_step

uint8[] compressed_data

geometry_msgs/Pose projection_center
