cmake_minimum_required(VERSION 3.0.2)
project(tie_robot_bringup)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch systemd
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
  scripts/install_frontend_autostart.sh
  scripts/install_rosbridge_service.sh
  scripts/install_driver_services.sh
  scripts/install_backend_service.sh
  scripts/install_show_legacy_frontend_service.sh
  scripts/install_demo_mode_service.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
