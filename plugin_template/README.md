# \<TEMPLATE POINT CLOUD TRANSPORT>
**ROS2 v0.1.**

This repository serves as a tutorial on how to create a custom plugin for `point_cloud_transport`. It assumes that you already have a working `point_cloud_transport` installation.

This is the spiritual successor to: https://github.com/paplhjak/templateplugin_point_cloud_transport and is heavily derived from it. In the ROS2 port, we opted to migrate the plugin tutorial but move it right next to the plugin code to avoid the instructions going stale. It is worth noting that there is nothing stopping the plugin you write from existing in a separate package or repository from `point_cloud_transport_plugins`.

## 1) Plugin naming
Each plugin needs its own name. It should be short and descriptive.

For example: [draco_point_cloud_transport](https://github.com/ros-perception/point_cloud_transport_plugins/tree/rolling/draco_point_cloud_transport) is a plugin which uses Google [Draco](https://github.com/google/draco) compression, therefore the name **draco**.

For demonstrative purposes, the plugin which we will be going through in this tutorial has used the name **template**. And we will be modifying the template to make our own plugin named **goblin**.

## 2) Setup your plugin package

Each transport plugin is its own ROS2 package. To create your own plugin package, let's clone the `point_cloud_transport_plugins` repo and make a copy of `plugin_template`:

```bash
$ cd /point_cloud_transport_ws/src
$ git clone https://github.com/ros-perception/point_cloud_transport_plugins.git
$ cp -r point_cloud_transport_plugins/plugin_template point_cloud_transport_plugins/goblin_point_cloud_transport
```

## 3) Time for Pattern Matching

This template uses the plugin name **template**, which is referenced repeatedly in the source files. The files and classes implemented within the plugin follow a strict naming convention, inheriting from the name of the plugin. To follow the convention, we will replace each instance of **template** with the name of our new plugin. This can be done quickly using a case-sensitive find and replace tool. Most editors can invoke this tool. Once you make sure that the find is case-sensitive, go through the files in *ONLY* the `goblin_point_cloud_transport` folder, and make the following replacements:

```
1. TEMPLATE -> GOBLIN
2. Template -> Goblin
3. template_ -> goblin_
4. "template" -> "goblin"
```

You can also rename the folders and files within the `goblin_point_cloud_transport` folder to match this convention. i.e.

```
1. *template_plugins.xml*
2. *src/template_publisher.cpp*
3. *src/template_subscriber.cpp*
4. *include/template_point_cloud_transport/template_publisher.hpp*
5. *include/template_point_cloud_transport/template_subscriber.hpp*
```

Be sure to double check that the `CMakeLists.txt` file points the expected files and that all the `#include`'s are still in order. Otherwise you will run into issues during the build phase.

## 4) Custom Message (Optional)

Although making your own compressed message type is an option, please check `point_cloud_interfaces` first to see if any existing compressed PointCloud2 message types already meet your needs. If they do, replace any instance of `CustomMessage` in the template package with that message's name, e.g. for `CompressedPointCloud2` (do not forget to check the #include's)

```
CustomMessage -> CompressedPointCloud2
```

If you do need a custom message, let's assume it is called **GobMessage**. See here if you are not familiar with defining a custom ROS2 message: https://docs.ros.org/en/rolling/index.html. 
Once you have defined your **GobMessage**, go through the following files (do not forget to check the #include's):

1. *goblin_publisher.h*
2. *goblin_subscriber.h*
3. *CMakeLists.txt*
4. *goblin_publisher.cpp*
5. *goblin_subscriber.cpp*
6. *goblin_plugins.xml*

and use the find and replace tool to replace the original name of **CustomMessage** with **GobMessage**.

```
CustomMessage -> GobMessage
```

## 5) Implementing Publisher Functionality

Implementation of the publisher can be located in *src/goblin_publisher.cpp* within function *encodeTyped*.

The encodeTyped function takes in a `sensor_msgs::msg::PointCloud2` message, compresses it and converts the compressed data into our plugin's message format (see Step 4). You might have noticed that the plugin is only concerned with the compression / conversion process and does not call publish. This was done intentionally to separate concerns between plugin implementation and core `point_cloud_transport` functionality.

## 6) Implementing Subscriber Functionality

Implementation of the subscriber can be located in *src/goblin_subscriber.cpp* within function *decodeTyped*.

The *decodeTyped* function takes in our plugin's message format, decompresses the data and converts it into a `sensor_msgs::msg::PointCloud2`. Once the point cloud message is ready, it is passed on to the subscriber callback. Just like in the publisher plugin, the subscriber plugin is only concerned with decoding and conversion. It does not actually call the subscriber callback.

## 7) Description of Plugin

Before we distribute our plugin, it is important that we fill in all the necessary information about it. 

In `goblin_plugins.xml`, make sure to provide a brief description of both the publisher and the subscriber our plugin uses.

Then fill out the package.xml. If you are unfamiliar with how to do this, see here please: https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html#customize-package-xml.

## 8) Build time!

At this point your plugin should be able to succesfully compile, build, and be recognized by [point_cloud_transport](https://github.com/ros-perception/point_cloud_transport). 

First, delete the COLCON_IGNORE file in `globlin_point_cloud_transport` (otherwise your package will be ignored).

Build the plugin.

```bash
$ cd /point_cloud_transport_ws
$ colcon build --merge-install --event-handlers console_direct+
```

Then check all plugins currently available on your system by running the command:

``` bash
source install/setup.bash
ros2 run point_cloud_transport list_transports
```

The output should look something like this.

```
"point_cloud_transport/goblin"
 - Provided by package: goblin_point_cloud_transport
 - Publisher: 
            This plugin publishes a CompressedPointCloud2 using the awesome power of turtles.
        
 - Subscriber: 
            This plugin decompresses a CompressedPointCloud2 topic, also using turtles.
```

Do you see your plugin? If not, please look back through these instructions and through the plugin code to verify you replaced all the instances of **template** with **goblin** and that the `CMakeLists.txt` and `package.xml` files are up to date w.r.t. file naming and any dependencies you have added.

Support
=======

If you have found an error in these instructions, please [file an issue](https://github.com/ros-perception/point_cloud_transport/issues).

Patches are encouraged, and may be submitted by forking this project and
submitting a pull request through GitHub. Any help is further development of the project is much appreciated.
