/*
 * Copyright (c) 2023, Czech Technical University in Prague
 * Copyright (c) 2019, paplhjak
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

#include <memory>
#include <string>
#include <vector>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl_point_cloud_transport/pcl_subscriber.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

namespace pcl_point_cloud_transport
{
void PCLSubscriber::declareParameters()
{
}

std::string PCLSubscriber::getTransportName() const
{
  return "pcl";
}

PCLSubscriber::DecodeResult PCLSubscriber::decodeTyped(
  const pcl_point_cloud_transport::PCLContainer & container) const
{
  auto message = std::make_shared<sensor_msgs::msg::PointCloud2>();

  pcl::PCLPointCloud2 pcl_pc2 = container.pcl();

  std::stringstream ss;
  ss << "output.pcd";
  std::cerr << "Data saved to " << ss.str() << std::endl;

  pcl::io::savePCDFile(
    ss.str(), pcl_pc2, Eigen::Vector4f::Zero(),
    Eigen::Quaternionf::Identity(), true);
    
  pcl_conversions::moveFromPCL(pcl_pc2, *message.get());

  return message;
  // pcl_point_cloud_transport::PCLContainer::ConstPtr l;
  // return message;
}

}  // namespace pcl_point_cloud_transport
