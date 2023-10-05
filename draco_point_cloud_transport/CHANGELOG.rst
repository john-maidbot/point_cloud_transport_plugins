^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package draco_point_cloud_transport
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.8 (2023-10-05)
------------------
* Fixed parameter names (`#28 <https://github.com/ros-perception/point_cloud_transport_plugins/issues/28>`_) (`#29 <https://github.com/ros-perception/point_cloud_transport_plugins/issues/29>`_)
  (cherry picked from commit 45c42b086cadb54ae88a102c6d3802589e267690)
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
* Contributors: mergify[bot]

1.0.7 (2023-09-21)
------------------
* use the right key for draco (`#21 <https://github.com/ros-perception/point_cloud_transport_plugins/issues/21>`_) (`#23 <https://github.com/ros-perception/point_cloud_transport_plugins/issues/23>`_)
  (cherry picked from commit c7b46c442317db84a876cfcafa2dd4b91696d236)
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
* feat: use tl_expected of ros package (`#22 <https://github.com/ros-perception/point_cloud_transport_plugins/issues/22>`_) (`#25 <https://github.com/ros-perception/point_cloud_transport_plugins/issues/25>`_)
  (cherry picked from commit ad4b632d977a8a06d641bd3fe1b21fc4ba8da0dd)
  Co-authored-by: Daisuke Nishimatsu <42202095+wep21@users.noreply.github.com>
* Contributors: mergify[bot]

1.0.6 (2023-09-20)
------------------
* ROS 2 port.
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
