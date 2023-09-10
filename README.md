# point_cloud_transport_plugins

This metapackage contains the most common plugins for pointcloud compression using [point_cloud_transport](https://wiki.ros.org/point_cloud_transport).

Currently provided are:

- [draco_point_cloud_transport](https://github.com/ros-perception/point_cloud_transport_plugins/tree/rolling/draco_point_cloud_transport) - A library using Google Draco to compress the pointclouds.

- [zlib_point_cloud_transport](https://github.com/ros-perception/point_cloud_transport_plugins/tree/rolling/zlib_point_cloud_transport) - A libraory using zlib to compress the pointclouds.

- [zstd_point_cloud_transport](https://github.com/ros-perception/point_cloud_transport_plugins/tree/master/zstd_point_cloud_transport) - A library using ZSTD to compress the pointclouds.

More transports can be added. Pull requests are welcome!

If it is not clear how to write a custom transport plugin, please see the `plugin_template` directory. This provides a template as well as written instructions for writing a custom transport plugin.
