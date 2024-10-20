cmake_minimum_required(VERSION 3.8)

# Enable AUTOMOC, AUTOUIC, AUTORCC for Qt
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

project(rviz2_cinematographer_gui)

# Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE RelWithDebInfo)

# Include the build directory for generated headers
include_directories(${CMAKE_CURRENT_BINARY_DIR})

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rqt_gui_cpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rviz2_cinematographer_view_controller REQUIRED)
find_package(rviz2_cinematographer_msgs REQUIRED)
# find_package(video_recorder REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# Source files
add_library(rviz2_cinematographer_gui_plugin SHARED
    src/plugins.cpp
    src/rviz2_cinematographer_gui.cpp
    src/resource.qrc
    src/rviz2_cinematographer_gui.ui
)

# Include directories
target_include_directories(rviz2_cinematographer_gui_plugin PUBLIC
    include
    Eigen3::Eigen
    PRIVATE ${CMAKE_CURRENT_BINARY_DIR}
)

# Add dependencies using ament_target_dependencies
ament_target_dependencies(rviz2_cinematographer_gui_plugin
    rclcpp
    interactive_markers
    nav_msgs
    rqt_gui_cpp
    pluginlib
    geometry_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    rviz2_cinematographer_view_controller
    rviz2_cinematographer_msgs
    yaml-cpp
    Eigen3
)

# Link against Qt and other non-ROS libraries manually
target_link_libraries(rviz2_cinematographer_gui_plugin
    Qt5::Widgets
    yaml-cpp
    Eigen3::Eigen
)

# Pluginlib export for the plugin description
pluginlib_export_plugin_description_file(rqt_gui_cpp plugin.xml)

# Install headers
install(
    DIRECTORY include/${PROJECT_NAME}/
    DESTINATION include/${PROJECT_NAME}
    PATTERN ".svn" EXCLUDE
)

# Install plugin description
install(FILES plugin.xml
    DESTINATION share/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

# Install trajectories directory
install(DIRECTORY trajectories
    DESTINATION share/${PROJECT_NAME}
)

# Install the library
install(TARGETS rviz2_cinematographer_gui_plugin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# Dummy target for IDEs
file(GLOB_RECURSE all_headers_for_ides
    ${CMAKE_SOURCE_DIR}/**/*.hpp
    ${CMAKE_SOURCE_DIR}/**/*.h
)
add_custom_target(ide_includes_${PROJECT_NAME} SOURCES ${all_headers_for_ides})

# Finalize the package
ament_package()
