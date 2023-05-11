// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague .. 2019, paplhjak

#pragma once

namespace draco_point_cloud_transport
{

//! copies header, width, ... between clouds
template<typename PC1, typename PC2>
void copyCloudMetadata(PC1& target, const PC2& source)
{
  target.header = source.header;
  target.height = source.height;
  target.width = source.width;
  target.fields = source.fields;
  target.is_bigendian = source.is_bigendian;
  target.point_step = source.point_step;
  target.row_step = source.row_step;
  target.is_dense = source.is_dense;
}

}
