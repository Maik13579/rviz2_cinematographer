/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "rviz2_cinematographer_view_controller/rviz2_cinematographer_view_controller.h"

namespace rviz2_cinematographer_view_controller
{

// Strings for selecting control mode styles
static const std::string MODE_ORBIT = "Orbit";
static const std::string MODE_FPS = "FPS";

// Limits to prevent orbit controller singularity, but not currently used.
static const Ogre::Radian PITCH_LIMIT_LOW  = Ogre::Radian(0.02);
static const Ogre::Radian PITCH_LIMIT_HIGH = Ogre::Radian(Ogre::Math::PI - 0.02);

// Some convenience functions for Ogre / geometry_msgs conversions
static inline Ogre::Vector3 vectorFromMsg(const geometry_msgs::msg::Point& m) { return Ogre::Vector3(m.x, m.y, m.z); }
static inline Ogre::Vector3 vectorFromMsg(const geometry_msgs::msg::Vector3& m) { return Ogre::Vector3(m.x, m.y, m.z); }

static inline geometry_msgs::msg::Point pointOgreToMsg(const Ogre::Vector3& o)
{
  geometry_msgs::msg::Point m;
  m.x = o.x; m.y = o.y; m.z = o.z;
  return m;
}

static inline geometry_msgs::msg::Vector3 vectorOgreToMsg(const Ogre::Vector3& o)
{
  geometry_msgs::msg::Vector3 m;
  m.x = o.x; m.y = o.y; m.z = o.z;
  return m;
}

// -----------------------------------------------------------------------------


CinematographerViewController::CinematographerViewController()
  : node_(rclcpp::Node::make_shared("rviz2_cinematographer_view_controller"))
    , animate_(false)
    , dragging_(false)
    , render_frame_by_frame_(false)
    , target_fps_(60)
    , recorded_frames_counter_(0)
    , do_wait_(false)
    , wait_duration_(-1.f)
{
  interaction_disabled_cursor_ = rviz_common::makeIconCursor(QPixmap("package://rviz2/icons/forbidden.svg"));


  mouse_enabled_property_ = new rviz_common::properties::BoolProperty("Mouse Enabled", true, "Enables mouse control of the camera.", this);

  interaction_mode_property_ = new rviz_common::properties::EditableEnumProperty("Control Mode", QString::fromStdString(MODE_ORBIT),
                                                        "Select the style of mouse interaction.", this);
  interaction_mode_property_->addOptionStd(MODE_ORBIT);
  interaction_mode_property_->addOptionStd(MODE_FPS);
  interaction_mode_property_->setStdString(MODE_ORBIT);

  fixed_up_property_ = new rviz_common::properties::BoolProperty("Maintain Vertical Axis", true, "If enabled, the camera is not allowed to roll side-to-side.", this);
  attached_frame_property_ = new rviz_common::properties::TfFrameProperty("Target Frame", rviz_common::properties::TfFrameProperty::FIXED_FRAME_STRING, "TF frame the camera is attached to.", this, nullptr, true);
  eye_point_property_ = new rviz_common::properties::VectorProperty("Eye", Ogre::Vector3(5, 5, 10), "Position of the camera.", this);
  focus_point_property_ = new rviz_common::properties::VectorProperty("Focus", Ogre::Vector3::ZERO, "Position of the focus/orbit point.", this);
  up_vector_property_ = new rviz_common::properties::VectorProperty("Up", Ogre::Vector3::UNIT_Z, "The vector which maps to \"up\" in the camera image plane.", this);
  distance_property_ = new rviz_common::properties::FloatProperty("Distance", getDistanceFromCameraToFocalPoint(), "The distance between the camera position and the focus point.", this);
  distance_property_->setMin(0.01);

  default_transition_duration_property_ = new rviz_common::properties::FloatProperty("Transition Duration in seconds", 0.5,
                                                        "The default duration to use for camera transitions.", this);
  camera_trajectory_topic_property_ = new rviz_common::properties::RosTopicProperty("Trajectory Topic", "/rviz2/camera_trajectory",
                                                           QString::fromStdString("rviz2_cinematographer_msgs/msg/CameraTrajectory"),
                                                           "Topic for CameraTrajectory messages", this,
                                                           SLOT(updateTopics()));

  transition_velocity_property_ = new rviz_common::properties::FloatProperty("Transition Velocity in m/s", 0, "The current velocity of the animated camera.", this);
  
  window_width_property_ = new rviz_common::properties::FloatProperty("Window Width", 1000, "The width of the rviz2 visualization window in pixels.", this);
  window_height_property_ = new rviz_common::properties::FloatProperty("Window Height", 1000, "The height of the rviz2 visualization window in pixels.", this);

  placement_pub_ = node_->create_publisher<geometry_msgs::msg::Pose>("/rviz2/current_camera_pose", 1);
  odometry_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("/rviz2/trajectory_odometry", 1);
  finished_rendering_trajectory_pub_ = node_->create_publisher<rviz2_cinematographer_msgs::msg::Finished>("/rviz2/finished_rendering_trajectory", 1);
  delete_pub_ = node_->create_publisher<std_msgs::msg::Empty>("/rviz2/delete", 1);

  image_pub_ = image_transport::create_publisher(node_.get(), "/rviz2/view_image");

  record_params_sub_ = node_->create_subscription<rviz2_cinematographer_msgs::msg::Record>(
    "/video_recorder/record_params", 1,
    [this](const rviz2_cinematographer_msgs::msg::Record::SharedPtr msg) { this->setRecord(msg); });

  wait_duration_sub_ = node_->create_subscription<rviz2_cinematographer_msgs::msg::Wait>(
      "/video_recorder/wait_duration", 1,
      [this](const rviz2_cinematographer_msgs::msg::Wait::SharedPtr msg) { this->setWaitDuration(msg); });

}

CinematographerViewController::~CinematographerViewController()
{
  context_->getSceneManager()->destroySceneNode(attached_scene_node_);
}

void CinematographerViewController::setRecord(rviz2_cinematographer_msgs::msg::Record::SharedPtr record_params)
{
  render_frame_by_frame_ = record_params->do_record > 0;

  int max_fps = 120;
  if(record_params->compress == 0)
    max_fps = 60;

  target_fps_ = std::max(1, std::min(max_fps, (int)record_params->frames_per_second));
}

void CinematographerViewController::setWaitDuration(const rviz2_cinematographer_msgs::msg::Wait::SharedPtr& wait_duration)
{
  wait_duration_ = wait_duration->seconds;
  do_wait_ = true;
}

void CinematographerViewController::updateTopics()
{
  trajectory_sub_ = node_->create_subscription<rviz2_cinematographer_msgs::msg::CameraTrajectory>(
      "camera_trajectory", rclcpp::QoS(10), [this](const rviz2_cinematographer_msgs::msg::CameraTrajectory::SharedPtr msg) {
          this->cameraTrajectoryCallback(msg);
      });

}

void CinematographerViewController::onInitialize()
{
  attached_frame_property_->setFrameManager(context_->getFrameManager());

  camera_->detachFromParent();
  attached_scene_node_ = context_->getSceneManager()->getRootSceneNode()->createChildSceneNode();
  attached_scene_node_->attachObject(camera_);
  camera_->setProjectionType(Ogre::PT_PERSPECTIVE);

  focal_shape_ = std::make_shared<rviz_rendering::Shape>(rviz_rendering::Shape::Sphere, context_->getSceneManager(), attached_scene_node_);
  focal_shape_->setScale(Ogre::Vector3(0.05f, 0.05f, 0.01f));
  focal_shape_->setColor(1.0f, 1.0f, 0.0f, 0.5f);
  focal_shape_->getRootNode()->setVisible(false);

  const unsigned long buffer_capacity = 100;
  cam_movements_buffer_ = BufferCamMovements(buffer_capacity);
  
  unsigned int width = context_->getViewManager()->getRenderPanel()->width();
  unsigned int height = context_->getViewManager()->getRenderPanel()->height();

}

void CinematographerViewController::onActivate()
{
  updateAttachedSceneNode();

  // Before activation, changes to target frame property should have
  // no side-effects.  After activation, changing target frame
  // property has the side effect (typically) of changing an offset
  // property so that the view does not jump.  Therefore we make the
  // signal/slot connection from the property here in onActivate()
  // instead of in the constructor.
  connect(attached_frame_property_, SIGNAL(changed()), this, SLOT(updateAttachedFrame()));
  connect(fixed_up_property_,       SIGNAL(changed()), this, SLOT(onUpPropertyChanged()));
  connectPositionProperties();

  updateTopics();
}

void CinematographerViewController::connectPositionProperties()
{
  connect(distance_property_,    SIGNAL(changed()), this, SLOT(onDistancePropertyChanged()), Qt::UniqueConnection);
  connect(eye_point_property_,   SIGNAL(changed()), this, SLOT(onEyePropertyChanged()),      Qt::UniqueConnection);
  connect(focus_point_property_, SIGNAL(changed()), this, SLOT(onFocusPropertyChanged()),    Qt::UniqueConnection);
  connect(up_vector_property_,   SIGNAL(changed()), this, SLOT(onUpPropertyChanged()),       Qt::UniqueConnection);
}

void CinematographerViewController::disconnectPositionProperties()
{
  disconnect(distance_property_,    SIGNAL(changed()), this, SLOT(onDistancePropertyChanged()));
  disconnect(eye_point_property_,   SIGNAL(changed()), this, SLOT(onEyePropertyChanged()));
  disconnect(focus_point_property_, SIGNAL(changed()), this, SLOT(onFocusPropertyChanged()));
  disconnect(up_vector_property_,   SIGNAL(changed()), this, SLOT(onUpPropertyChanged()));
}

void CinematographerViewController::onEyePropertyChanged()
{
  distance_property_->setFloat(getDistanceFromCameraToFocalPoint());
}

void CinematographerViewController::onFocusPropertyChanged()
{
  distance_property_->setFloat(getDistanceFromCameraToFocalPoint());
}

void CinematographerViewController::onDistancePropertyChanged()
{
  disconnectPositionProperties();
  Ogre::Vector3 new_eye_position =
    focus_point_property_->getVector() + distance_property_->getFloat() * camera_->getDerivedOrientation().zAxis();
  eye_point_property_->setVector(new_eye_position);
  connectPositionProperties();
}

void CinematographerViewController::onUpPropertyChanged()
{
  disconnect(up_vector_property_, SIGNAL(changed()), this, SLOT(onUpPropertyChanged()));
  if(fixed_up_property_->getBool())
  {
    up_vector_property_->setVector(Ogre::Vector3::UNIT_Z);
    Ogre::Quaternion yaw_axis_quat(Ogre::Radian(0), Ogre::Vector3::UNIT_Y);
    camera_->setOrientation(yaw_axis_quat);

  }
  else
  {
    // force orientation to match up vector; first call doesn't actually change the quaternion
    camera_->setFixedYawAxis(true, reference_orientation_ * up_vector_property_->getVector());
    camera_->setDirection(
      reference_orientation_ * (focus_point_property_->getVector() - eye_point_property_->getVector()));
    // restore normal behavior
    camera_->setFixedYawAxis(false, Ogre::Vector3::UNIT_Y);
  }
  connect(up_vector_property_, SIGNAL(changed()), this, SLOT(onUpPropertyChanged()), Qt::UniqueConnection);
}

void CinematographerViewController::updateAttachedFrame()
{
  Ogre::Vector3 old_position = attached_scene_node_->_getDerivedPosition();
  Ogre::Quaternion old_orientation = attached_scene_node_->_getDerivedOrientation();

  updateAttachedSceneNode();

  onAttachedFrameChanged(old_position, old_orientation);
}

void CinematographerViewController::updateAttachedSceneNode()
{
  std::string error_msg;
  if(!context_->getFrameManager()->transformHasProblems(attached_frame_property_->getFrameStd(), rclcpp::Time(), error_msg))
  {
    context_->getFrameManager()->getTransform(attached_frame_property_->getFrameStd(), rclcpp::Time(), reference_position_,
                                              reference_orientation_);
    attached_scene_node_->setPosition(reference_position_);
    attached_scene_node_->setOrientation(reference_orientation_);
    context_->queueRender();
  }
  else
  {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "Transform not available: %s", error_msg.c_str());

  }
}

void CinematographerViewController::onAttachedFrameChanged(const Ogre::Vector3& old_reference_position,
                                                           const Ogre::Quaternion& old_reference_orientation)
{
  Ogre::Vector3 fixed_frame_focus_position =
    old_reference_orientation * focus_point_property_->getVector() + old_reference_position;
  Ogre::Vector3 fixed_frame_eye_position =
    old_reference_orientation * eye_point_property_->getVector() + old_reference_position;
  Ogre::Vector3 new_focus_position = fixedFrameToAttachedLocal(fixed_frame_focus_position);
  Ogre::Vector3 new_eye_position = fixedFrameToAttachedLocal(fixed_frame_eye_position);
  Ogre::Vector3 new_up_vector =
    reference_orientation_.Inverse() * old_reference_orientation * up_vector_property_->getVector();

  focus_point_property_->setVector(new_focus_position);
  eye_point_property_->setVector(new_eye_position);
  up_vector_property_->setVector(fixed_up_property_->getBool() ? Ogre::Vector3::UNIT_Z : new_up_vector);
  distance_property_->setFloat(getDistanceFromCameraToFocalPoint());

  // force orientation to match up vector; first call doesn't actually change the quaternion
  camera_->setFixedYawAxis(true, reference_orientation_ * up_vector_property_->getVector());
  camera_->setDirection(reference_orientation_ * (focus_point_property_->getVector() - eye_point_property_->getVector()));
}

float CinematographerViewController::getDistanceFromCameraToFocalPoint()
{
  return (eye_point_property_->getVector() - focus_point_property_->getVector()).length();
}

void CinematographerViewController::reset()
{
  eye_point_property_->setVector(Ogre::Vector3(5, 5, 10));
  focus_point_property_->setVector(Ogre::Vector3::ZERO);
  up_vector_property_->setVector(Ogre::Vector3::UNIT_Z);
  distance_property_->setFloat(getDistanceFromCameraToFocalPoint());
  mouse_enabled_property_->setBool(true);
  interaction_mode_property_->setStdString(MODE_ORBIT);

  // Hersh says: why is the following junk necessary?  I don't know.
  // However, without this you need to call reset() twice after
  // switching from TopDownOrtho to FPS.  After the first call the
  // camera is in the right position but pointing the wrong way.
  updateCamera();
  camera_->lookAt(Ogre::Vector3(0, 0, 0));
  setPropertiesFromCamera(camera_);
}

void CinematographerViewController::handleMouseEvent(rviz_common::ViewportMouseEvent& evt)
{
  if(!mouse_enabled_property_->getBool())
  {
    setCursor(interaction_disabled_cursor_);
    setStatus("<b>Mouse interaction is disabled. You can enable it by checking the \"Mouse Enabled\" check-box in the Views panel.");
    return;
  }
  else if(evt.shift())
  {
    setStatus("TODO: Fix me! <b>Left-Click:</b> Move X/Y.  <b>Right-Click:</b>: Move Z.");
  }
  else if(evt.control())
  {
    setStatus("TODO: Fix me! <b>Left-Click:</b> Move X/Y.  <b>Right-Click:</b>: Move Z.");
  }
  else
  {
    setStatus("TODO: Fix me! <b>Left-Click:</b> Rotate.  <b>Middle-Click:</b> Move X/Y.  <b>Right-Click:</b>: Zoom.  <b>Shift</b>: More options.");
  }

  float distance = distance_property_->getFloat();
  int32_t diff_x = 0;
  int32_t diff_y = 0;
  bool moved = false;

  if(evt.type == QEvent::MouseButtonPress)
  {
    focal_shape_->getRootNode()->setVisible(true);
    moved = true;
    dragging_ = true;
    cancelTransition();  // Stop any automated movement
  }
  else if(evt.type == QEvent::MouseButtonRelease)
  {
    focal_shape_->getRootNode()->setVisible(false);
    moved = true;
    dragging_ = false;
  }
  else if(dragging_ && evt.type == QEvent::MouseMove)
  {
    diff_x = evt.x - evt.last_x;
    diff_y = evt.y - evt.last_y;
    moved = true;
  }

  // regular left-button drag
  if(evt.left() && !evt.shift())
  {
    setCursor(Rotate3D);
    yaw_pitch_roll(-diff_x * 0.005f, -diff_y * 0.005f, 0);
  }
  // middle or shift-left drag
  else if(evt.middle() || (evt.shift() && evt.left()))
  {
    setCursor(MoveXY);
    if(interaction_mode_property_->getStdString() == MODE_ORBIT)  // Orbit style
    {
      float fovY = camera_->getFOVy().valueRadians();
      float fovX = 2.0f * static_cast<float>(atan(tan(fovY / 2.0) * camera_->getAspectRatio()));

      int width = camera_->getViewport()->getWidth();
      int height = camera_->getViewport()->getHeight();

      move_focus_and_eye(-((float)diff_x / width) * distance * static_cast<float>(tan(fovX / 2.0)) * 2.0f,
                         ((float)diff_y / height) * distance * static_cast<float>(tan(fovY / 2.0)) * 2.0f,
                         0.0f);
    }
    else if(interaction_mode_property_->getStdString() == MODE_FPS)  // FPS style
    {
      move_focus_and_eye(diff_x * 0.01f, -diff_y * 0.01f, 0.0f);
    }
  }
  else if(evt.right())
  {
    if(evt.shift() || (interaction_mode_property_->getStdString() == MODE_FPS))
    {
      setCursor(MoveZ);
      move_focus_and_eye(0.0f, 0.0f, diff_y * 0.01f * distance);
    }
    else
    {
      setCursor(Zoom);
      move_eye(0, 0, diff_y * 0.01f * distance);
    }
  }
  else
  {
    setCursor(evt.shift() ? MoveXY : Rotate3D);
  }

  if(evt.wheel_delta != 0)
  {
    int diff = evt.wheel_delta;

    if(evt.shift())
      move_focus_and_eye(0, 0, -diff * 0.001f * distance);
    else if(evt.control())
      yaw_pitch_roll(0, 0, diff * 0.001f);
    else
      move_eye(0, 0, -diff * 0.001f * distance);
    moved = true;
  }

  if(evt.type == QEvent::MouseButtonPress && evt.left() && evt.control() && evt.shift())
  {
    bool was_orbit = (interaction_mode_property_->getStdString() == MODE_ORBIT);
    interaction_mode_property_->setStdString(was_orbit ? MODE_FPS : MODE_ORBIT);
  }

  if(moved)
  {
    publishCameraPose();
    context_->queueRender();
  }
}

void CinematographerViewController::handleKeyEvent(QKeyEvent* event, rviz_common::RenderPanel* panel)
{
  if(event->key() == 16777223) // press on delete button
  {
    std_msgs::msg::Empty delete_msg;
    delete_pub_->publish(delete_msg);
  }
}

void CinematographerViewController::setPropertiesFromCamera(Ogre::Camera* source_camera)
{
  disconnectPositionProperties();
  Ogre::Vector3 direction = source_camera->getDerivedOrientation() * Ogre::Vector3::NEGATIVE_UNIT_Z;
  eye_point_property_->setVector(source_camera->getDerivedPosition());
  focus_point_property_->setVector(source_camera->getDerivedPosition() + direction * distance_property_->getFloat());
  if(fixed_up_property_->getBool())
    up_vector_property_->setVector(Ogre::Vector3::UNIT_Z);
  else
    up_vector_property_->setVector(source_camera->getDerivedOrientation().yAxis());

  connectPositionProperties();
}

void CinematographerViewController::mimic(ViewController* source_view)
{
  QVariant target_frame = source_view->subProp("Target Frame")->getValue();
  if(target_frame.isValid())
    attached_frame_property_->setValue(target_frame);

  Ogre::Camera* source_camera = source_view->getCamera();
  Ogre::Vector3 position = source_camera->getDerivedPosition();
  Ogre::Quaternion orientation = source_camera->getDerivedOrientation();

  if(source_view->getClassId() == "rviz2/Orbit")
    distance_property_->setFloat(source_view->subProp("Distance")->getValue().toFloat());
  else
    distance_property_->setFloat(position.length());

  interaction_mode_property_->setStdString(MODE_ORBIT);

  Ogre::Vector3 direction = orientation * (Ogre::Vector3::NEGATIVE_UNIT_Z * distance_property_->getFloat());
  focus_point_property_->setVector(position + direction);
  eye_point_property_->setVector(position);

  updateCamera();
}

void CinematographerViewController::transitionFrom(ViewController* previous_view)
{
  auto prev_view_controller = dynamic_cast<CinematographerViewController*>(previous_view);
  if(prev_view_controller)
  {
    Ogre::Vector3 new_eye = eye_point_property_->getVector();
    Ogre::Vector3 new_focus = focus_point_property_->getVector();
    Ogre::Vector3 new_up = up_vector_property_->getVector();

    eye_point_property_->setVector(prev_view_controller->eye_point_property_->getVector());
    focus_point_property_->setVector(prev_view_controller->focus_point_property_->getVector());
    up_vector_property_->setVector(prev_view_controller->up_vector_property_->getVector());

    beginNewTransition(new_eye,
                       new_focus,
                       new_up,
                       rclcpp::Duration::from_nanoseconds(static_cast<int64_t>(default_transition_duration_property_->getFloat() * 1e9)));
  }
}

void CinematographerViewController::beginNewTransition(const Ogre::Vector3& eye,
                                                       const Ogre::Vector3& focus,
                                                       const Ogre::Vector3& up,
                                                       rclcpp::Duration transition_duration,
                                                       uint8_t interpolation_speed)
{
  // if jump was requested, perform as usual but prevent division by zero
  if (transition_duration.nanoseconds() == 0)
    transition_duration = rclcpp::Duration::from_nanoseconds(static_cast<int64_t>(0.001 * 1e9));

  // if the buffer is empty we set the first element in it to the current camera pose
  if(cam_movements_buffer_.empty())
  {
    rclcpp::Clock clock(RCL_SYSTEM_TIME);
    transition_start_time_ = clock.now();

    cam_movements_buffer_.push_back(std::move(OgreCameraMovement(
                                              eye_point_property_->getVector(),
                                              focus_point_property_->getVector(),
                                              up_vector_property_->getVector(),
                                              rclcpp::Duration::from_nanoseconds(static_cast<int64_t>(0.001 * 1e9)),
                                              interpolation_speed)));

  }

  if(cam_movements_buffer_.full())
    cam_movements_buffer_.set_capacity(cam_movements_buffer_.capacity() + 20);

  cam_movements_buffer_.push_back(std::move(OgreCameraMovement(eye, focus, up, transition_duration, interpolation_speed)));

  animate_ = true;
}

void CinematographerViewController::cancelTransition()
{
  animate_ = false;
  cam_movements_buffer_.clear();
  recorded_frames_counter_ = 0;

  if(render_frame_by_frame_)
  {
    rviz2_cinematographer_msgs::msg::Finished finished;
    finished.is_finished = true;
    finished_rendering_trajectory_pub_->publish(finished);
    render_frame_by_frame_ = false;
  }
}

void CinematographerViewController::cameraTrajectoryCallback(const rviz2_cinematographer_msgs::msg::CameraTrajectory::SharedPtr ct_ptr)
{
  rviz2_cinematographer_msgs::msg::CameraTrajectory ct = *ct_ptr;

  if(ct.trajectory.empty())
    return;

  // Handle control parameters
  mouse_enabled_property_->setBool(!ct.interaction_disabled);
  fixed_up_property_->setBool(!ct.allow_free_yaw_axis);
  if(ct.mouse_interaction_mode != rviz2_cinematographer_msgs::msg::CameraTrajectory::NO_CHANGE)
  {
    std::string name = "";
    if(ct.mouse_interaction_mode == rviz2_cinematographer_msgs::msg::CameraTrajectory::ORBIT)
      name = MODE_ORBIT;
    else if(ct.mouse_interaction_mode == rviz2_cinematographer_msgs::msg::CameraTrajectory::FPS)
      name = MODE_FPS;
    interaction_mode_property_->setStdString(name);
  }

  for(auto& cam_movement : ct.trajectory)
  {
    transformCameraMovementToAttachedFrame(cam_movement);

    if(ct.target_frame != "")
    {
      attached_frame_property_->setStdString(ct.target_frame);
      updateAttachedFrame();
    }

    Ogre::Vector3 eye = vectorFromMsg(cam_movement.eye.point);
    Ogre::Vector3 focus = vectorFromMsg(cam_movement.focus.point);
    Ogre::Vector3 up = vectorFromMsg(cam_movement.up.vector);
    beginNewTransition(eye, focus, up, cam_movement.transition_duration, cam_movement.interpolation_speed);
  }
}

void CinematographerViewController::transformCameraMovementToAttachedFrame(rviz2_cinematographer_msgs::msg::CameraMovement& cm)
{
  Ogre::Vector3 position_fixed_eye, position_fixed_focus, position_fixed_up;
  Ogre::Quaternion rotation_fixed_eye, rotation_fixed_focus, rotation_fixed_up;

  context_->getFrameManager()->getTransform(cm.eye.header.frame_id,   rclcpp::Time(0), position_fixed_eye,   rotation_fixed_eye);
  context_->getFrameManager()->getTransform(cm.focus.header.frame_id, rclcpp::Time(0), position_fixed_focus, rotation_fixed_focus);
  context_->getFrameManager()->getTransform(cm.up.header.frame_id,    rclcpp::Time(0), position_fixed_up,    rotation_fixed_up);

  Ogre::Vector3 eye = vectorFromMsg(cm.eye.point);
  Ogre::Vector3 focus = vectorFromMsg(cm.focus.point);
  Ogre::Vector3 up = vectorFromMsg(cm.up.vector);

  eye = fixedFrameToAttachedLocal(position_fixed_eye + rotation_fixed_eye * eye);
  focus = fixedFrameToAttachedLocal(position_fixed_focus + rotation_fixed_focus * focus);
  up = reference_orientation_.Inverse() * rotation_fixed_up * up;

  cm.eye.point = pointOgreToMsg(eye);
  cm.focus.point = pointOgreToMsg(focus);
  cm.up.vector = vectorOgreToMsg(up);
  cm.eye.header.frame_id = attached_frame_property_->getStdString();
  cm.focus.header.frame_id = attached_frame_property_->getStdString();
  cm.up.header.frame_id = attached_frame_property_->getStdString();
}

// We must assume that this point is in the rviz2 Fixed frame since it came from rviz2...
void CinematographerViewController::lookAt(const Ogre::Vector3& point)
{
  if(!mouse_enabled_property_->getBool()) return;

  Ogre::Vector3 new_point = fixedFrameToAttachedLocal(point);

  beginNewTransition(eye_point_property_->getVector(),
                     new_point,
                     up_vector_property_->getVector(),
                     rclcpp::Duration::from_nanoseconds(static_cast<int64_t>(default_transition_duration_property_->getFloat() * 1e9)));
}

void CinematographerViewController::publishOdometry(const Ogre::Vector3& position,
                                                    const Ogre::Vector3& velocity)
{
  nav_msgs::msg::Odometry odometry;
  odometry.header.frame_id = attached_frame_property_->getFrameStd();
  odometry.header.stamp = node_->now();
  odometry.pose.pose.position.x = position.x;
  odometry.pose.pose.position.y = position.y;
  odometry.pose.pose.position.z = position.z;
  odometry.twist.twist.linear.x = velocity.x; //This is allo velocity and therefore not ROS convention!
  odometry.twist.twist.linear.y = velocity.y; //This is allo velocity and therefore not ROS convention!
  odometry.twist.twist.linear.z = velocity.z; //This is allo velocity and therefore not ROS convention!

  Ogre::Quaternion cam_orientation = camera_->getDerivedOrientation();
  Ogre::Quaternion rot_around_y_pos_90_deg(0.707f, 0.0f, 0.707f, 0.0f);
  cam_orientation = cam_orientation * rot_around_y_pos_90_deg;
  odometry.pose.pose.orientation.x = cam_orientation.x;
  odometry.pose.pose.orientation.y = cam_orientation.y;
  odometry.pose.pose.orientation.z = cam_orientation.z;
  odometry.pose.pose.orientation.w = cam_orientation.w;
  odometry_pub_->publish(odometry);
}

float CinematographerViewController::computeRelativeProgressInSpace(double relative_progress_in_time,
                                                                    uint8_t interpolation_speed)
{
  switch(interpolation_speed)
  {
    case rviz2_cinematographer_msgs::msg::CameraMovement::RISING:
      return 1.f - static_cast<float>(cos(relative_progress_in_time * M_PI_2));
    case rviz2_cinematographer_msgs::msg::CameraMovement::DECLINING:
      return static_cast<float>(-cos(relative_progress_in_time * M_PI_2 + M_PI_2));
    case rviz2_cinematographer_msgs::msg::CameraMovement::FULL:
      return static_cast<float>(relative_progress_in_time);
    case rviz2_cinematographer_msgs::msg::CameraMovement::WAVE:
    default:
      return 0.5f * (1.f - static_cast<float>(cos(relative_progress_in_time * M_PI)));
  }
}

void CinematographerViewController::update(float dt, float ros_dt)
{
  updateAttachedSceneNode();

  // there has to be at least two positions in the buffer - start and goal
  if(animate_ && cam_movements_buffer_.size() > 1)
  {
    auto start = cam_movements_buffer_.begin();
    auto goal = ++(cam_movements_buffer_.begin());

    double relative_progress_in_time = 0.0;
    if(render_frame_by_frame_)
    {
      relative_progress_in_time = recorded_frames_counter_ / (target_fps_ * goal->transition_duration.seconds());
      recorded_frames_counter_++;
    }
    else
    {
      rclcpp::Clock clock(RCL_SYSTEM_TIME);
      rclcpp::Duration duration_from_start = clock.now() - transition_start_time_;
      relative_progress_in_time = duration_from_start.seconds() / goal->transition_duration.seconds();
    }

    // make sure we get all the way there before turning off
    if(relative_progress_in_time >= 1.0)
    {
      relative_progress_in_time = 1.0;
      animate_ = false;
    }

    float relative_progress_in_space = computeRelativeProgressInSpace(relative_progress_in_time,
                                                                      goal->interpolation_speed);

    Ogre::Vector3 new_position = start->eye + relative_progress_in_space * (goal->eye - start->eye);
    Ogre::Vector3 new_focus = start->focus + relative_progress_in_space * (goal->focus - start->focus);
    Ogre::Vector3 new_up = start->up + relative_progress_in_space * (goal->up - start->up);

    Ogre::Vector3 velocity = (new_position - eye_point_property_->getVector()) / ros_dt;
    transition_velocity_property_->setFloat(velocity.normalise());

    if(odometry_pub_->get_subscription_count() != 0)
      publishOdometry(new_position, velocity);

    disconnectPositionProperties();
    eye_point_property_->setVector(new_position);
    focus_point_property_->setVector(new_focus);
    up_vector_property_->setVector(new_up);
    distance_property_->setFloat(getDistanceFromCameraToFocalPoint());
    connectPositionProperties();

    // This needs to happen so that the camera orientation will update properly when fixed_up_property == false
    camera_->setFixedYawAxis(true, reference_orientation_ * up_vector_property_->getVector());
    camera_->setDirection(reference_orientation_ * (focus_point_property_->getVector() - eye_point_property_->getVector()));

    publishCameraPose();

    if(render_frame_by_frame_ && image_pub_.getNumSubscribers() > 0)
      publishViewImage();

    // if current movement is over
    if(!animate_)
    {
      // delete current start element in buffer
      cam_movements_buffer_.pop_front();
      recorded_frames_counter_ = 0;

      // if there are still movements to perform
      if(cam_movements_buffer_.size() > 1)
      {
        // reset animate to perform the next movement
        animate_ = true;
        // update the transition start time with the duration the transition should have taken
        transition_start_time_ += rclcpp::Duration::from_seconds(cam_movements_buffer_.front().transition_duration.seconds());
      }
      else
      {
        // clean up
        cam_movements_buffer_.clear();

        // publish that the rendering is finished 
        if(render_frame_by_frame_)
        {
          rviz2_cinematographer_msgs::msg::Finished finished;
          finished.is_finished = true;
          // wait a little so last image is send before this "finished"-message 
          rclcpp::sleep_for(std::chrono::nanoseconds(static_cast<int64_t>(1.0 / target_fps_ * 1e9)));
          finished_rendering_trajectory_pub_->publish(finished);
          render_frame_by_frame_ = false;
        }
      }
    }
  }
  else
    transition_velocity_property_->setFloat(0.f);

  updateCamera();
  
  window_width_property_->setFloat(context_->getViewManager()->getRenderPanel()->width());
  window_height_property_->setFloat(context_->getViewManager()->getRenderPanel()->height());
}

void CinematographerViewController::publishViewImage()
{
  // wait for specified duration - e.g. if recorder is not fast enough
  if(do_wait_)
  {
    rclcpp::sleep_for(std::chrono::nanoseconds(static_cast<int64_t>(1.0 / wait_duration_ * 1e9)));
    do_wait_ = false;
  }

  unsigned int width = context_->getViewManager()->getRenderPanel()->width();
  unsigned int height = context_->getViewManager()->getRenderPanel()->height();


  // create a PixelBox of the needed size to store the rendered image
  Ogre::PixelFormat format = Ogre::PF_BYTE_BGR;
  auto outBytesPerPixel = Ogre::PixelUtil::getNumElemBytes(format);
  auto data = new unsigned char[width * height * outBytesPerPixel];
  Ogre::Box extents(0, 0, width, height);
  Ogre::PixelBox pb(extents, format, data);
  Ogre::TexturePtr texture = Ogre::TextureManager::getSingleton().createManual(
    "RenderTexture",
    Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
    Ogre::TEX_TYPE_2D,
    width,
    height,
    0,
    Ogre::PF_R8G8B8,
    Ogre::TU_RENDERTARGET);

  Ogre::RenderTexture* render_texture = texture->getBuffer()->getRenderTarget();
  render_texture->update();
  render_texture->copyContentsToMemory(pb, Ogre::RenderTarget::FB_FRONT);


  // convert the image in the PixelBox to a sensor_msgs::msg::Image and publish
  auto ros_image = std::make_shared<sensor_msgs::msg::Image>();
  ros_image->header.frame_id = attached_frame_property_->getStdString();
  ros_image->header.stamp = node_->get_clock()->now();
  ros_image->height = height;
  ros_image->width = width;
  ros_image->encoding = sensor_msgs::image_encodings::BGR8;
  ros_image->is_bigendian = false;
  ros_image->step = static_cast<unsigned int>(width * outBytesPerPixel);
  size_t size = width * outBytesPerPixel * height;
  ros_image->data.resize(size);
  memcpy((char*)(&ros_image->data[0]), data, size);

  image_pub_.publish(ros_image);

  delete[] data;
}

void CinematographerViewController::updateCamera()
{
  camera_->setPosition(eye_point_property_->getVector());
  camera_->setFixedYawAxis(fixed_up_property_->getBool(), reference_orientation_ * up_vector_property_->getVector());
  camera_->setDirection(reference_orientation_ * (focus_point_property_->getVector() - eye_point_property_->getVector()));

  focal_shape_->setPosition(focus_point_property_->getVector());
}

void CinematographerViewController::publishCameraPose()
{
  geometry_msgs::msg::Pose cam_pose;
  cam_pose.position.x = camera_->getDerivedPosition().x;
  cam_pose.position.y = camera_->getDerivedPosition().y;
  cam_pose.position.z = camera_->getDerivedPosition().z;
  cam_pose.orientation.w = camera_->getDerivedOrientation().w;
  cam_pose.orientation.x = camera_->getDerivedOrientation().x;
  cam_pose.orientation.y = camera_->getDerivedOrientation().y;
  cam_pose.orientation.z = camera_->getDerivedOrientation().z;
  placement_pub_->publish(cam_pose);
}

void CinematographerViewController::yaw_pitch_roll(float yaw, float pitch, float roll)
{
  Ogre::Quaternion old_camera_orientation = camera_->getDerivedOrientation();
  Ogre::Radian old_pitch = old_camera_orientation.getPitch(false);

  if(fixed_up_property_->getBool())
    yaw = static_cast<float>(cos(old_pitch.valueRadians() - Ogre::Math::HALF_PI)) * yaw; // helps to reduce crazy spinning!

  Ogre::Quaternion yaw_quat, pitch_quat, roll_quat;
  yaw_quat.FromAngleAxis(Ogre::Radian(yaw), Ogre::Vector3::UNIT_Y);
  pitch_quat.FromAngleAxis(Ogre::Radian(pitch), Ogre::Vector3::UNIT_X);
  roll_quat.FromAngleAxis(Ogre::Radian(roll), Ogre::Vector3::UNIT_Z);
  Ogre::Quaternion orientation_change = yaw_quat * pitch_quat * roll_quat;
  Ogre::Quaternion new_camera_orientation = old_camera_orientation * orientation_change;
  Ogre::Radian new_pitch = new_camera_orientation.getPitch(false);

  if(fixed_up_property_->getBool() &&
     ((new_pitch > PITCH_LIMIT_HIGH && new_pitch > old_pitch) ||
      (new_pitch < PITCH_LIMIT_LOW && new_pitch < old_pitch)))
  {
    orientation_change = yaw_quat * roll_quat;
    new_camera_orientation = old_camera_orientation * orientation_change;
  }

  camera_->setOrientation(new_camera_orientation);
  if(interaction_mode_property_->getStdString() == MODE_ORBIT)
  {
    // In orbit mode the focal point stays fixed, so we need to compute the new camera position.
    Ogre::Vector3 new_eye_position =
      focus_point_property_->getVector() + distance_property_->getFloat() * new_camera_orientation.zAxis();
    eye_point_property_->setVector(new_eye_position);
    camera_->setPosition(new_eye_position);
    setPropertiesFromCamera(camera_);
  }
  else
  {
    // In FPS mode the camera stays fixed, so we can just apply the rotations and then rely on the property update to set the new focal point.
    setPropertiesFromCamera(camera_);
  }
}

void CinematographerViewController::move_focus_and_eye(const float x,
                                                       const float y,
                                                       const float z)
{
  Ogre::Vector3 translate(x, y, z);
  eye_point_property_->add(camera_->getDerivedOrientation() * translate);
  focus_point_property_->add(camera_->getDerivedOrientation() * translate);
}

void CinematographerViewController::move_eye(const float x,
                                             const float y,
                                             const float z)
{
  Ogre::Vector3 translate(x, y, z);

  // Only update the camera position if it won't "pass through" the origin
  Ogre::Vector3 new_position = eye_point_property_->getVector() + camera_->getDerivedOrientation() * translate;
  if((new_position - focus_point_property_->getVector()).length() > distance_property_->getMin())
    eye_point_property_->setVector(new_position);
  distance_property_->setFloat(getDistanceFromCameraToFocalPoint());
}


} // end namespace rviz2

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rviz2_cinematographer_view_controller::CinematographerViewController, rviz_common::ViewController)

