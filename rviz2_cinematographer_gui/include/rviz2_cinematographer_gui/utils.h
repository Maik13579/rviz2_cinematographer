/** @file
 *
 * Helper functions for plugins.
 *
 * @authors 
 *  - Jan Razlaw
 *  - Maik Knof (port to ROS2)
 */

#ifndef RVIZ2_CINEMATOGRAPHER_GUI_UTILS_H
#define RVIZ2_CINEMATOGRAPHER_GUI_UTILS_H

#include <rclcpp/rclcpp.hpp>
#include <cmath> // For M_SQRT1_2

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace rviz2_cinematographer_gui
{

/**
 * @brief Creates a box marker.
 *
 * @param[in] scale Scale factor for the marker.
 * @return Box marker.
 */
inline visualization_msgs::msg::Marker makeBox(float scale)
{
  visualization_msgs::msg::Marker marker;
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.pose.orientation.w = M_SQRT1_2;
  marker.pose.orientation.y = M_SQRT1_2;
  marker.scale.x = scale * 0.15f;
  marker.scale.y = scale * 0.45f;
  marker.scale.z = scale * 0.25f;
  marker.color.r = 0.f;
  marker.color.g = 0.f;
  marker.color.b = 0.f;
  marker.color.a = 1.f;
  return marker;
}

/**
 * @brief Creates an arrow marker.
 *
 * @param[in] scale Scale factor for the marker.
 * @return Arrow marker.
 */
inline visualization_msgs::msg::Marker makeArrow(float scale)
{
  visualization_msgs::msg::Marker marker;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.pose.orientation.w = M_SQRT1_2;
  marker.pose.orientation.y = M_SQRT1_2;
  marker.scale.x = scale * 0.7f;
  marker.scale.y = scale * 0.1f;
  marker.scale.z = scale * 0.1f;
  marker.color.r = 1.f;
  marker.color.g = 1.f;
  marker.color.b = 1.f;
  marker.color.a = 0.6f;
  return marker;
}

/**
 * @brief Augments an interactive marker with box and arrow controls.
 *
 * @param[in,out] marker Interactive marker to be augmented with controls.
 */
inline void makeBoxControl(visualization_msgs::msg::InteractiveMarker& marker)
{
  visualization_msgs::msg::InteractiveMarkerControl control;
  control.always_visible = true;
  control.orientation.w = 1.0f;
  
  // Assuming 'scale.x' is representative of the overall scale
  float scale = marker.scale.x;

  control.markers.push_back(makeBox(scale));
  control.markers.push_back(makeArrow(scale));
  
  marker.controls.push_back(control);
  marker.controls.back().interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::BUTTON;
  marker.controls.back().name = "submit_button";
}

/**
 * @brief Finds a parameter name that contains the given substring.
 *
 * @param[in] node ROS2 node handle.
 * @param[in,out] param_name Substring to search for. Updated to the full parameter name if found.
 * @return true If a matching parameter is found and param_name is updated.
 * @return false Otherwise.
 */
inline bool getFullParamName(const rclcpp::Node::SharedPtr& node,
                             std::string& param_name)
{
  std::vector<std::string> prefixes;
  prefixes.emplace_back("*" + param_name + "*");
  
  // Search for parameters that match the pattern
  rcl_interfaces::msg::ListParametersResult result = node->list_parameters(prefixes, 10);
  
  if (!result.names.empty()) {
    param_name = result.names[0];
    return true;
  }
  
  return false;
}

/**
 * @brief Retrieves a parameter value, updating the parameter name if a match is found.
 *
 * @tparam T Type of the parameter.
 * @param[in] node ROS2 node handle.
 * @param[in,out] param_name Substring to search for and update to the full name.
 * @param[out] param Reference to store the parameter value.
 * @param[in] default_param Default value if the parameter is not found.
 * @return true If the parameter was found and retrieved.
 * @return false If the parameter was not found and the default was used.
 */
template<typename T>
inline bool getParam(const rclcpp::Node::SharedPtr& node,
                    std::string& param_name,
                    T& param,
                    T default_param)
{
  bool found = getFullParamName(node, param_name);
  
  if (found) {
    // Ensure the parameter exists before trying to get it
    if (node->has_parameter(param_name)) {
      node->get_parameter(param_name, param);
      return true;
    } else {
      RCLCPP_WARN(node->get_logger(), "Parameter '%s' was found by name pattern but does not exist.", param_name.c_str());
    }
  }
  
  // Parameter not found; assign default value
  param = default_param;
  return false;
}

} // namespace rviz2_cinematographer_gui

#endif // RVIZ2_CINEMATOGRAPHER_GUI_UTILS_H
