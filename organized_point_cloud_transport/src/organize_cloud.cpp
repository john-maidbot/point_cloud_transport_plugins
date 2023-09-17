/*
 * Copyright (c) 2023, John D'Angelo
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *
 *    * Neither the name of the copyright holder nor the names of its
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <string>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <organized_point_cloud_transport/organized_publisher.hpp>

namespace organized_point_cloud_transport
{
  void OrganizedPublisher::organizePointCloud2(const sensor_msgs::msg::PointCloud2 &unorganized_cloud, sensor_msgs::msg::PointCloud2 &organized_cloud) const
  {
    // get the transform between the frame_id of the pointcloud and the frame_id of the view point
    geometry_msgs::msg::TransformStamped transform_stamped;
    if (unorganized_cloud.header.frame_id == projector_info_.header.frame_id)
    {
      transform_stamped.transform.rotation.w = 1;
    }
    else
    {
      try
      {
        // impatiently lookup the transform
        transform_stamped = tf_buffer_->lookupTransform(
            projector_info_.header.frame_id, unorganized_cloud.header.frame_id, unorganized_cloud.header.stamp, rclcpp::Duration::from_seconds(0.1));
      }
      catch (tf2::TransformException &ex)
      {
        RCLCPP_ERROR_STREAM(getLogger(), "Failed to lookup transform from " << projector_info_.header.frame_id << " to " << unorganized_cloud.header.frame_id << "!");
        RCLCPP_ERROR_STREAM(getLogger(), "Using identity transform instead!");
      }
    }

    // Check if the cloud has rgb data
    const auto predicate = [](const auto &field)
    { return field.name == "rgb"; };
    const auto result = std::find_if(unorganized_cloud.fields.cbegin(), unorganized_cloud.fields.cend(), predicate);
    const bool has_rgb = result != unorganized_cloud.fields.cend();

    // initialize the pointcloud accordingly
    organized_cloud.is_dense = true;
    organized_cloud.header.frame_id = projector_info_.header.frame_id;
    organized_cloud.header.stamp = unorganized_cloud.header.stamp;
    organized_cloud.height = projector_info_.height;
    organized_cloud.width = projector_info_.width;
    organized_cloud.is_bigendian = unorganized_cloud.is_bigendian;

    sensor_msgs::PointCloud2Modifier modifier(organized_cloud);
    if (has_rgb)
    {
      modifier.setPointCloud2FieldsByString(2, "xyz", "rgb"); // Define fields as XYZ and RGB
    }
    else
    {
      modifier.setPointCloud2FieldsByString(1, "xyz"); // Define fields as XYZ
    }
    modifier.resize(organized_cloud.width * organized_cloud.height);

    // create iterators for accessing the unorganized pointcloud data
    sensor_msgs::PointCloud2ConstIterator<float> u_iter_x(unorganized_cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> u_iter_y(unorganized_cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> u_iter_z(unorganized_cloud, "z");
    sensor_msgs::PointCloud2ConstIterator<uint8_t> u_iter_r(unorganized_cloud, "r");
    sensor_msgs::PointCloud2ConstIterator<uint8_t> u_iter_g(unorganized_cloud, "g");
    sensor_msgs::PointCloud2ConstIterator<uint8_t> u_iter_b(unorganized_cloud, "b");

    // initialize the point objects for the transform
    geometry_msgs::msg::PointStamped old_point, new_point;

    for (; u_iter_x != u_iter_x.end();)
    {
      old_point.point.x = *u_iter_x;
      old_point.point.y = *u_iter_y;
      old_point.point.z = *u_iter_z;

      // skip points that are not finite
      if (!std::isfinite(old_point.point.x) || !std::isfinite(old_point.point.y) || !std::isfinite(old_point.point.z))
      {
        continue;
      }

      // transform the point into the view point frame
      tf2::doTransform(old_point, new_point, transform_stamped);

      if (new_point.point.z <= 0)
      {
        // the imaginary camera doesnt see this point
        continue;
      }

      const int col = new_point.point.x / new_point.point.z * projector_info_.k[0] + projector_info_.k[2];
      const int row = new_point.point.y / new_point.point.z * projector_info_.k[4] + projector_info_.k[5];

      if (col < 0 || row < 0 || col >= projector_info_.width || row >= projector_info_.height)
      {
        // the imaginary camera doesnt see this point
        continue;
      }

      // get index into organized pointcloud
      const int index = row * organized_cloud.row_step + col * organized_cloud.point_step;

      // get the current z coordinate of this cell in the organized pointcloud
      // reinterprety the uint8_t memory as float
      float *z_ptr = reinterpret_cast<float *>(&organized_cloud.data[index + 2 * 4]);

      if (*z_ptr < new_point.point.z)
      {
        // this point is further than the one already in the image at this pixel
        continue;
      }

      if (has_rgb)
      {
        // copy the xyz and rgb values
        *reinterpret_cast<float *>(&organized_cloud.data[index + 0 * 4]) = new_point.point.x;
        *reinterpret_cast<float *>(&organized_cloud.data[index + 1 * 4]) = new_point.point.y;
        *z_ptr = new_point.point.z;
        // TODO (john-maidbot): This might be wrong
        organized_cloud.data[index + 3 * 4 + 1] = *u_iter_r;
        organized_cloud.data[index + 3 * 4 + 2] = *u_iter_g;
        organized_cloud.data[index + 3 * 4 + 3] = *u_iter_b;
        ++u_iter_r, ++u_iter_g, ++u_iter_b;
      }
      else
      {
        *reinterpret_cast<float *>(&organized_cloud.data[index + 0 * 4]) = new_point.point.x;
        *reinterpret_cast<float *>(&organized_cloud.data[index + 1 * 4]) = new_point.point.y;
        *z_ptr = new_point.point.z;
      }
      ++u_iter_x, ++u_iter_y, ++u_iter_z;
    }
  }
} // namespace organized_point_cloud_transport
