/** @file
 *
 * Simple rqt plugin to edit trajectories.
 *
 * @author Jan Razlaw
 * @author Maik Knof (port to ROS2)
 */

#ifndef RVIZ2_CINEMATOGRAPHER_GUI_H
#define RVIZ2_CINEMATOGRAPHER_GUI_H

#include <filesystem>
namespace fs = std::filesystem;

#include <fstream>
#include <sstream>
#include <string>
#include <unistd.h>
#include <signal.h>

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

#include <rviz2_cinematographer_msgs/msg/camera_movement.hpp>
#include <rviz2_cinematographer_msgs/msg/camera_trajectory.hpp>
#include <rviz2_cinematographer_msgs/msg/record.hpp>
#include <rviz2_cinematographer_msgs/msg/finished.hpp>

#include <std_msgs/msg/empty.hpp>

#include <nav_msgs/msg/path.hpp>

#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>

#include <rqt_gui_cpp/plugin.h>

#include <QWidget>
#include <QFileDialog>

#include <rviz2_cinematographer_gui/utils.h>
#include "ui_rviz2_cinematographer_gui.h"

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/thread.hpp>
#include <yaml-cpp/yaml.h>

#include <spline_library/splines/natural_spline.h>
#include <spline_library/splines/uniform_cr_spline.h>
#include <spline_library/vector.h>


namespace rviz2_cinematographer_gui
{

/**
 * @brief Manipulates the rviz2 camera.
 */
class Rviz2CinematographerGUI : public rqt_gui_cpp::Plugin
{

Q_OBJECT
public:
  struct InteractiveMarkerWithDurations
  {
    InteractiveMarkerWithDurations(visualization_msgs::msg::InteractiveMarker&& input_marker,
                                   const double transition_duration,
                                   const double wait_duration = 0.0)
      : marker(input_marker)
        , transition_duration(transition_duration)
        , wait_duration(wait_duration)
    {
    }

    visualization_msgs::msg::InteractiveMarker marker;
    double transition_duration;
    double wait_duration;
  };

  typedef InteractiveMarkerWithDurations TimedMarker;
  typedef std::list<TimedMarker> MarkerList;
  typedef typename MarkerList::iterator MarkerIterator;

 enum
  {
    RISING_INTERPOLATION_SPEED = rviz2_cinematographer_msgs::msg::CameraMovement::RISING,
    DECLINING_INTERPOLATION_SPEED = rviz2_cinematographer_msgs::msg::CameraMovement::DECLINING,
    FULL_INTERPOLATION_SPEED = rviz2_cinematographer_msgs::msg::CameraMovement::FULL,
    WAVE_INTERPOLATION_SPEED = rviz2_cinematographer_msgs::msg::CameraMovement::WAVE,
  };


  /** @brief Constructor. */
  Rviz2CinematographerGUI();
  ~Rviz2CinematographerGUI() = default;

  /**
   * @brief Sets up subscribers and publishers and connects GUI to functions.
   *
   * @param context     the plugin context.
   */
  virtual void initPlugin(qt_gui_cpp::PluginContext& context);

  /**
   * @brief Shuts down the subscribers and publishers.
   */
  virtual void shutdownPlugin();

  /**
   * @brief Saves settings. TODO.
   *
   * @param plugin_settings     plugin-specific settings
   * @param instance_settings   instance-specific settings
   */
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings,
                            qt_gui_cpp::Settings& instance_settings) const;

  /**
   * @brief Restores settings. TODO.
   *
   * @param plugin_settings     plugin-specific settings
   * @param instance_settings   instance-specific settings
   */
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
                               const qt_gui_cpp::Settings& instance_settings);

  /**
   * @brief Saves the current camera pose to #cam_pose_.
   *
   * @param[in] cam_pose    pointer to current camera pose.
   */
  void camPoseCallback(const geometry_msgs::msg::Pose::ConstPtr& cam_pose);

Q_SIGNALS:
  void updateRequested();

public slots:
  /** @brief Moves rviz2 camera to currently selected pose.*/
  void moveCamToCurrent();
  /** @brief Moves rviz2 camera to the pose before the selected one.*/
  void moveCamToPrev();
  /** @brief Moves rviz2 camera subsequently to the first pose in the trajectory.*/
  void moveCamToFirst();
  /** @brief Moves rviz2 camera to the pose after the selected one.*/
  void moveCamToNext();
  /** @brief Moves rviz2 camera subsequently to the last pose in the trajectory.*/
  void moveCamToLast();
  /** @brief Update selected marker with values from GUI.*/
  void updateCurrentMarker();
  /** @brief Updates marker durations on change in table with values from table.*/
  void updateMarker();
  /** @brief Updates which marker is labeled as the currently active marker.*/
  void updateWhoIsCurrentMarker(int marker_id);
  /** @brief Appends the current pose of the rviz2 camera to the trajectory.*/
  void appendCamPoseToTrajectory();
  /** @brief Sets selected pose to the current pose of the rviz2 camera.*/
  void setCurrentPoseToCam();
  /** @brief Reconstructs trajectory from current markers. */
  void updateTrajectory();
  /** @brief Sets the frame_id of the markers.*/
  void setMarkerFrames();
  /** @brief Increase the scale of the markers.*/
  void increaseMarkerScale();
  /** @brief Decrease the scale of the markers.*/
  void decreaseMarkerScale();
  /** @brief Show/Hide interactive marker controls.*/
  void showInteractiveMarkerControls();
  /** @brief Loads a series of markers from a file.*/
  void loadTrajectoryFromFile();
  /** @brief Saves poses and transition durations of interactive markers to a file.*/
  void saveTrajectoryToFile();
  /** @brief Opens file explorer to specify the path of the recorded video.*/
  void setVideoOutputPath();
  /** @brief Adds a marker between the currently selected marker and the one before in the trajectory.*/
  void addMarkerBefore();
  /** @brief Adds a marker at the pose of the selected marker.*/
  void addMarkerHere();
  /** @brief Adds a marker between the selected marker and the next one in the trajectory.*/
  void addMarkerBehind();
  /** @brief Removes the current marker.*/
  void removeCurrentMarker();
  /** @brief Fill time table with values from markers.*/
  void refillTable();
  
private:
  /**
   * @brief Creates a CameraMovement hull.
   * @return CameraMovement.
   */
  rviz2_cinematographer_msgs::msg::CameraMovement makeCameraMovement();

  /**
   * @brief Creates an InteractiveMarker hull.
   *
   * @param[in] x   x position of marker.
   * @param[in] y   y position of marker.
   * @param[in] z   z position of marker.
   * @return InteractiveMarker.
   */
  visualization_msgs::msg::InteractiveMarker makeMarker(double x = 0.0,
                                                        double y = 0.0,
                                                        double z = 0.0);

  /**
   * @brief Colorize all markers in red.
   */
  void colorizeMarkersRed();

  /**
    * @brief Create and fill time table.
    */
  void setUpTimeTable();

  /** @brief Updates the scale of the provided marker.
   *
   * @param[in, out]    marker          marker that is updated
   * @param[in]         scale_factor    factor used to update the marker scale
   */
  void updateMarkerScale(TimedMarker& marker,
                         float scale_factor);

  /** @brief Updates the scales of the markers.
   *
   * @param[in]     scale_factor   factor used to update the markers' scales
   */
  void updateMarkerScales(float scale_factor);

  /**
   * @brief Checks if current position of rviz2 camera is within bounds of spin boxes.
   *
   * @return true, if camera is within bounds.
   */
  bool isCamWithinBounds();

  /**
   * @brief Rotates rviz2_cam_pose around z-axis for -90 degrees.
   *
   * @param[in]     rviz2_cam_pose   camera orientation as defined by rviz2.
   * @param[out]    marker_pose     camera orientation as defined by markers.
   */
  void rviz2CamToMarkerOrientation(const geometry_msgs::msg::Pose& rviz2_cam_pose,
                                  geometry_msgs::msg::Pose& marker_pose);

  /**
   * @brief Creates cam movement to goal marker and appends this to the trajectory. 
   *
   * Default interpolation behaviour: Increase speed up to next marker, stay at full speed until next-to-last marker 
   * reached, decrease speed then 
   * Special Cases: - Omit full speed if halting directly after the next marker 
   *                - Increase and decrease in one motion (wave) if halting at next marker  
   *
   * @param[in]         goal_marker_iter   defines where to extend the trajectory to.
   * @param[in,out]     cam_trajectory     trajectory that is extended.
   * @param[in]         last_marker_iter   iterator to last marker of marker list.
   */
  void appendMarkerToTrajectory(const MarkerIterator& goal_marker_iter,
                                rviz2_cinematographer_msgs::msg::CameraTrajectoryPtr& cam_trajectory,
                                const MarkerIterator& last_marker_iter);

  /**
   * @brief Fills a CameraMovement message with the values of a TimedMarker.
   *
   * @param[in]     marker          marker.
   * @param[out]    cam_movement    message.
   */
  void convertMarkerToCamMovement(const TimedMarker& marker,
                                  rviz2_cinematographer_msgs::msg::CameraMovement& cam_movement);

  /**
   * @brief Moves rviz2 camera to marker pose by publishing a CameraTrajectory message.
   *
   * @param[in] marker_name         name of marker.
   * @param[in] transition_duration     (optional) provide time needed to get to marker - default: transition_duration of marker.
   */
  void moveCamToMarker(const std::string& marker_name,
                       double transition_duration = -1.0);

  /**
   * @brief Updates members using pose of currently moved interactive marker.
   *
   * @param[in] feedback    feedback the interaction with the interactive marker generates.
   */
  void processFeedback(const visualization_msgs::msg::InteractiveMarkerFeedbackConstPtr& feedback);

  /**
   * @brief Rotates a vector by a quaternion.
   *
   * @param[in] vector  input vector.
   * @param[in] quat    input rotation.
   * @return the rotated vector.
   */
  tf::Vector3 rotateVector(const tf::Vector3& vector,
                           const geometry_msgs::msg::Quaternion& quat);

  /**
   * @brief Safes poses and durations of markers to yaml file.
   *
   * @param[in] file_path   path to the file.
   */
  void safeTrajectoryToFile(const std::string& file_path);

  /**
   * @brief Adds a marker between the selected marker and the one before in the trajectory.
   *
   * @param[in] feedback    feedback from selected marker.
   */
  void addMarkerBeforeClicked(const visualization_msgs::msg::InteractiveMarkerFeedbackConstPtr& feedback);

  /**
   * @brief Adds a marker between the currently selected marker and the one before in the trajectory.
   *
   * @param[in] current_marker_name    name of currently selected marker.
   */
  void addMarkerBefore(const std::string& current_marker_name);

  /**
   * @brief Adds a marker at the pose of the selected marker.
   *
   * @param[in] feedback    feedback from selected marker.
   */
  void addMarkerAtClicked(const visualization_msgs::msg::InteractiveMarkerFeedbackConstPtr& feedback);
  
  /**
   * @brief Adds a marker at the pose of the selected marker.
   *
   * @param[in] current_marker_name    name of currently selected marker.
   */
  void addMarkerHere(const std::string& current_marker_name);

  /**
   * @brief Adds a marker between the selected marker and the next one in the trajectory.
   *
   * @param[in] feedback    feedback from selected marker.
   */
  void addMarkerBehindClicked(const visualization_msgs::msg::InteractiveMarkerFeedbackConstPtr& feedback);

  /**
   * @brief Adds a marker between the selected marker and the next one in the trajectory.
   *
   * @param[in] current_marker_name    name of currently selected marker.
   */
  void addMarkerBehind(const std::string& current_marker_name);

  /**
   * @brief Removes the selected marker.
   *
   * @param[in] feedback    feedback from selected marker.
   */
  void removeClickedMarker(const visualization_msgs::msg::InteractiveMarkerFeedbackConstPtr& feedback);
  
  /**
   * @brief Removes the marker called marker_name.
   *
   * @param[in] marker_name    name of marker that should be removed.
   */
  void removeMarker(const std::string& marker_name);

  /**
   * @brief Removes the current marker.
   *
   * @param[in] empty    empty msgs needed as this is a callback function.
   */
  void removeCurrentMarker(const std_msgs::msg::EmptyConstPtr& empty);

  /**
   * @brief Saves markers in server.
   *
   * @param[in] markers     markers.
   */
  void updateServer(MarkerList& markers);

  /**
   * @brief Load marker poses from a file.
   *
   * @param[in] node        node.
   * @param[in] param_name  root name of the parameters.
   */
  void loadParams(const rclcpp::Node::SharedPtr node, const std::string& param_name);

  /**
   * @brief Gets marker with specified name.
   *
   * @param[in] marker_name name of marker.
   * @return marker with marker_name.
   */
  InteractiveMarkerWithDurations& getMarkerByName(const std::string& marker_name);

  /**
   * @brief Sets the #current_marker_ to the provided input.
   *
   * Additionally updates the GUI elements and sets the color of the input marker to green.
   *
   * @param[in] marker  input marker.
   */
  void setCurrentTo(TimedMarker& marker);

  /**
   * @brief Resets the #current_marker_ and dependencies from old_current to new_current.
   *
   * Dependencies are the marker member, the marker server and the GUI.
   *
   * @param[in,out] old_current  marker that was current before.
   * @param[in,out] new_current  marker that is current now.
   */
  void setCurrentFromTo(TimedMarker& old_current,
                        TimedMarker& new_current);

  /**
   * @brief Sets the pose and durations in the GUI to the provided input values.
   *
   * @param[in] current_marker      currently active marker providing values for GUI
   */
  void updateGUIValues(const TimedMarker& current_marker);

  /**
   * @brief Sets the value of the spin_box to the value without triggering a signal.
   *
   * @param[in,out] spin_box    the updated spin box.
   * @param[in]     value       the value the spin box is set to.
   */
  void setValueQuietly(QDoubleSpinBox* spin_box,
                       double value);

  /**
   * @brief Interpolate markers using a spline and safe that spline in spline_poses.
   *
   * @param[in]     markers         markers to be interpolated.
   * @param[out]    spline_poses    constructed spline.
   * @param[in]     frequency       resolution - number of spline points between each marker pair.
   * @param[in]     duplicate_ends  flag if first and last marker should be duplicated for spline - some types of splines don't interpolate between the the end points and the ones next to them.
   */
  void markersToSplinedPoses(const MarkerList& markers,
                             std::vector<geometry_msgs::msg::Pose>& spline_poses,
                             double frequency,
                             bool duplicate_ends = true);

  /**
   * @brief Interpolate markers using a spline and safe that spline as the points of a CameraTrajectory.
   *
   * @param[in]     markers         markers to be interpolated.
   * @param[out]    trajectory      resulting trajectory.
   */
  void markersToSplinedCamTrajectory(const MarkerList& markers,
                                     rviz2_cinematographer_msgs::msg::CameraTrajectoryPtr trajectory);

  /**
   * @brief Generates trajectories for eye positions, focus positions and up directories, needed for spline generation.
   *
   * @param[in]     markers                   markers defining trajectory.
   * @param[out]    input_eye_positions       camera positions at trajectory points.
   * @param[out]    input_focus_positions     positions of camera focus at trajectory points.
   * @param[out]    input_up_directions       cameras up directions at trajectory points.
   */
  void prepareSpline(const MarkerList& markers,
                     std::vector<Vector3>& input_eye_positions,
                     std::vector<Vector3>& input_focus_positions,
                     std::vector<Vector3>& input_up_directions);

  /**
   * @brief Computes transition durations for each step within the spline.
   *
   * @param[in]     markers                     markers defining trajectory.
   * @param[out]    transition_durations        transition durations between spline points.
   * @param[out]    wait_durations              wait durations at spline points.
   * @param[out]    total_transition_duration   sum of all transition durations.
   */
  void computeDurations(const MarkerList& markers,
                        std::vector<double>& transition_durations,
                        std::vector<double>& wait_durations,
                        double& total_transition_duration);

  /**
   * @brief Convert spline to CameraTrajectory.
   *
   * @param[in]     eye_spline                  spline of camera positions.
   * @param[in]     focus_spline                spline of camera focus points.
   * @param[in]     up_spline                   spline of camera up positions.
   * @param[in]     transition_durations        transition duration between spline points.
   * @param[in]     wait_durations              wait duration at spline points.
   * @param[in]     total_transition_duration   overall transition duration.
   * @param[out]    trajectory                  resulting camera trajectory.
   */
  void splineToCamTrajectory(const UniformCRSpline<Vector3>& eye_spline,
                             const UniformCRSpline<Vector3>& focus_spline,
                             const UniformCRSpline<Vector3>& up_spline,
                             const std::vector<double>& transition_durations,
                             const std::vector<double>& wait_durations,
                             const double total_transition_duration,
                             rviz2_cinematographer_msgs::CameraTrajectoryPtr trajectory);

  /** @brief Call service to record current trajectory. */
  void publishRecordParams();

  /** @brief Listen to the message that the recording is over. */
  void recordFinishedCallback(const rviz2_cinematographer_msgs::msg::Finished::ConstPtr& record_finished);

  /** @brief Starts video recorder nodelet. */
  void videoRecorderThread();

  /** @brief Returns index of marker with marker_name. */
  int getMarkerId(const std::string& marker_name){return std::stoi(marker_name) - 1;};

  /** @brief Clicks the button.
   * 
   * Detour is necessary because the time table has to be updated which is only possible from the main thread
   */
  void clickButton(QPushButton* button){button->click();};
  
  /** @brief Ui object - connection to GUI. */
  Ui::rviz2_cinematographer_gui ui_;
  /** @brief Widget. */
  QWidget* widget_;

  rclcpp::Publisher<rviz2_cinematographer_msgs::msg::CameraTrajectory>::SharedPtr camera_trajectory_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr view_poses_array_pub_;
  rclcpp::Publisher<rviz2_cinematographer_msgs::msg::Record>::SharedPtr record_params_pub_;

  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr camera_pose_sub_;
  rclcpp::Subscription<rviz2_cinematographer_msgs::msg::Finished>::SharedPtr record_finished_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr delete_marker_sub_;

  std::shared_ptr<std::thread> video_recorder_thread_;

  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  interactive_markers::MenuHandler menu_handler_;

  geometry_msgs::msg::Pose cam_pose_;
  std::string current_marker_name_;
  MarkerList markers_;
  bool recorder_running_;
};

} // namespace

#endif //RVIZ2_CINEMATOGRAPHER_GUI_H
