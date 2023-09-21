^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package draco_point_cloud_transport
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.1 (2023-09-21)
------------------
* feat: use tl_expected of ros package (`#22 <https://github.com/ros-perception/point_cloud_transport_plugins/issues/22>`_) (`#26 <https://github.com/ros-perception/point_cloud_transport_plugins/issues/26>`_)
  (cherry picked from commit ad4b632d977a8a06d641bd3fe1b21fc4ba8da0dd)
  Co-authored-by: Daisuke Nishimatsu <42202095+wep21@users.noreply.github.com>
* Contributors: mergify[bot]

2.0.0 (2023-09-18)
------------------
* Port to ROS 2
* Contributors: Alejandro Hernandez Cordero, john-maidbot

1.0.5 (2023-06-16)
------------------
* Fixed formatting.
* Contributors: Martin Pecka

1.0.4 (2023-06-16)
------------------
* Fixed handling of clouds with invalid points.
* Contributors: Martin Pecka

1.0.3 (2023-05-13)
------------------
* Compatibility with GCC 7.
* Contributors: Martin Pecka

1.0.2 (2023-05-11)
------------------
* Report log messages from the transport via the log helper.
* Hide draco library symbols from include files so that it does not have to be exported.
* Contributors: Martin Pecka

1.0.1 (2023-05-11)
------------------
* More robust Draco CMake finding.
* Adopted direct encoders and decoders.
* Moved draco_point_cloud_transport into its own folder
* Forked from https://github.com/paplhjak/draco_point_cloud_transport
* Contributors: Martin Pecka, Jakub Paplham, Tomas Petricek
