// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague .. 2019, paplhjak

#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <draco/point_cloud/point_cloud.h>
#include <draco/point_cloud/point_cloud_builder.h>

#include <sensor_msgs/PointCloud2.h>

class PC2toDraco
{
public:
  //! Constructor.
  explicit PC2toDraco(sensor_msgs::PointCloud2 PC2, std::string topic);

  //! Method for converting into Draco pointcloud
  std::unique_ptr<draco::PointCloud> convert(bool deduplicate_flag, bool expert_encoding_flag);

private:
  //! Message to be converted
  sensor_msgs::PointCloud2 PC2_;

  //! enumeration for switch(std::string)
  enum StringValueFieldName
  {
    enumvalGeneric,
    enumval1,
    enumval2,
    enumval3,
    enumval4,
    enumval5,
    enumval6,
    enumval7,
    enumval8,
    enumval9,
    enumval10,
    enumval11,
    enumval12,
    enumval13,
    enumval14,
    enumvalEnd
  };

  std::map<std::string, StringValueFieldName> s_mapStringValues;

  //! initialize enumeration values for detection of GeometryAttribute::Type
  void Initialize()
  {
    s_mapStringValues["x"] = enumval1;
    s_mapStringValues["y"] = enumval2;
    s_mapStringValues["z"] = enumval3;
    s_mapStringValues["pos"] = enumval4;
    s_mapStringValues["position"] = enumval5;
    s_mapStringValues["r"] = enumval6;
    s_mapStringValues["g"] = enumval7;
    s_mapStringValues["b"] = enumval8;
    s_mapStringValues["a"] = enumval9;
    s_mapStringValues["rgb"] = enumval10;
    s_mapStringValues["rgba"] = enumval11;
    s_mapStringValues["nx"] = enumval12;
    s_mapStringValues["ny"] = enumval13;
    s_mapStringValues["nz"] = enumval14;
  }

  std::string base_topic_;
};
