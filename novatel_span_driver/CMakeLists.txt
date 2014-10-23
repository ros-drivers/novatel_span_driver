cmake_minimum_required(VERSION 2.8.3)
project(novatel_span_driver)

find_package(catkin REQUIRED COMPONENTS rospy)

catkin_python_setup()

catkin_package()


install(PROGRAMS scripts/driver
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
