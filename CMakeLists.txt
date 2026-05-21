cmake_minimum_required(VERSION 3.16)
project(robotdynid_ros2)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(robotdynid_ros2)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/robotdynid/robotdynid/__init__.py")
  ament_python_install_package(
    robotdynid
    PACKAGE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/robotdynid/robotdynid"
  )
else()
  message(FATAL_ERROR "robotdynid submodule is missing. Run: git submodule update --init --recursive")
endif()

install(
  PROGRAMS
    scripts/dataset_recorder
    scripts/robotdynid-bag-to-csv
    scripts/robotdynid-cancel-trajectory
    scripts/robotdynid-export-runtime
    scripts/robotdynid-gui
    scripts/robotdynid-generate-excitation
    scripts/robotdynid-identify-codegen
    scripts/robotdynid-preprocess-dataset
    scripts/robotdynid-preview-trajectory
    scripts/robotdynid-send-trajectory
    scripts/robotdynid-validate-dataset
    scripts/robotdynid-validate-excitation
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    config
    examples
    launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(robotdynid_ros2_pytest test)
endif()

ament_package()
