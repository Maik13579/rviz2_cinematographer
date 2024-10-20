cmake_minimum_required(VERSION 3.8)
project(rviz2_cinematographer_view_controller)

# Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(pluginlib REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(OpenGL REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(rviz2_cinematographer_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(rviz_rendering REQUIRED) # Add rviz_rendering

# Qt wrapping
qt5_wrap_cpp(MOC_FILES
  include/${PROJECT_NAME}/rviz2_cinematographer_view_controller.h
)

# Build the library
add_library(${PROJECT_NAME} SHARED
  src/rviz2_cinematographer_view_controller.cpp
  ${MOC_FILES}
)

# Include directories using target_include_directories
target_include_directories(${PROJECT_NAME} PUBLIC
  include
  ${Boost_INCLUDE_DIRS}
  ${rviz_ogre_vendor_INCLUDE_DIRS}
  ${rviz_rendering_INCLUDE_DIRS} # Add rendering include
  ${Qt5Widgets_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${rviz_common_INCLUDE_DIRS}
)

# Link libraries
target_link_libraries(${PROJECT_NAME}
  ${rviz_ogre_vendor_LIBRARIES}
  ${rviz_rendering_LIBRARIES} # Add rendering library
  Qt5::Widgets
  ${Boost_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${rviz_common_LIBRARIES}
  ${cv_bridge_LIBRARIES}
  ${OpenCV_LIBRARIES}
  image_transport::image_transport  # Added for image_transport
)

# Add dependencies for correct ordering
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  rviz_common
  rviz_rendering # Add rendering here
  rviz_default_plugins
  rviz2_cinematographer_msgs
  Boost
  cv_bridge
  OpenCV
  Eigen3
  pluginlib
  rviz_ogre_vendor
  image_transport
  image_geometry
)

# Pluginlib export for the plugin description
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

# Install headers
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  PATTERN ".svn" EXCLUDE
)

# Install plugin description
install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# Install the library
install(TARGETS ${PROJECT_NAME}
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
