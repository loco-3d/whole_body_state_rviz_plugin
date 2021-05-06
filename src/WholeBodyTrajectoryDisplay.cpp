///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2020, University of Edinburgh, Istituto Italiano di Tecnologia
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "whole_body_state_rviz_plugin/WholeBodyTrajectoryDisplay.h"
#include "whole_body_state_rviz_plugin/PinocchioLinkUpdater.h"
#include <Eigen/Dense>
#include <OgreManualObject.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <QTimer>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/parsers/urdf.hpp>

using namespace rviz;

namespace whole_body_state_rviz_plugin {

void linkUpdaterStatusFunction(rviz::StatusLevel level,
                               const std::string &link_name,
                               const std::string &text,
                               WholeBodyTrajectoryDisplay *display) {
  display->setStatus(level, QString::fromStdString(link_name),
                     QString::fromStdString(text));
}

WholeBodyTrajectoryDisplay::WholeBodyTrajectoryDisplay()
    : is_info_(false), weight_(0.), target_enable_(true), com_enable_(true),
      com_axes_enable_(true), contact_enable_(true),
      contact_axes_enable_(true) {
  // Category Groups
  target_category_ = new rviz::Property("Target", QVariant(), "", this);
  com_category_ = new rviz::Property("Center of Mass", QVariant(), "", this);
  contact_category_ = new rviz::Property("End-Effector", QVariant(), "", this);

  // Target properties
  target_enable_property_ =
      new BoolProperty("Enable", true, "Enable/disable the Target display",
                       target_category_, SLOT(updateTargetEnable()), this);
  robot_description_property_ = new StringProperty(
      "Robot Description", "robot_description",
      "Name of the parameter to search for to load the robot description.",
      target_category_, SLOT(updateRobotDescription()), this);
  robot_visual_enabled_property_ =
      new Property("Robot Visual", true,
                   "Whether to display the visual representation of the robot.",
                   target_category_, SLOT(updateRobotVisualVisible()), this);
  robot_collision_enabled_property_ = new Property(
      "Robot Collision", false,
      "Whether to display the collision representation of the robot.",
      target_category_, SLOT(updateRobotCollisionVisible()), this);
  robot_alpha_property_ = new FloatProperty(
      "Robot Alpha", 0.2, "Amount of transparency to apply to the links.",
      target_category_, SLOT(updateRobotAlpha()), this);
  robot_alpha_property_->setMin(0.0);
  robot_alpha_property_->setMax(1.0);
  force_color_property_ = new ColorProperty(
      "Force Color", QColor(85, 0, 255), "Color to draw the arrow.",
      target_category_, SLOT(updateForceColorAndAlpha()), this);
  force_alpha_property_ = new FloatProperty(
      "Force Alpha", 0.2, "Amount of transparency to apply to the arrow.",
      target_category_, SLOT(updateForceColorAndAlpha()), this);
  force_alpha_property_->setMin(0);
  force_alpha_property_->setMax(1);
  force_shaft_length_property_ = new FloatProperty(
      "Force Shaft Length", 0.8, "Length of the arrow's shaft, in meters.",
      target_category_, SLOT(updateForceArrowGeometry()), this);
  force_shaft_radius_property_ = new FloatProperty(
      "Force Shaft Radius", 0.02, "Radius of the arrow's shaft, in meters.",
      target_category_, SLOT(updateForceArrowGeometry()), this);
  force_head_length_property_ = new FloatProperty(
      "Force Head Length", 0.08, "Length of the arrow's head, in meters.",
      target_category_, SLOT(updateForceArrowGeometry()), this);
  force_head_radius_property_ = new FloatProperty(
      "Force Head Radius", 0.04, "Radius of the arrow's head, in meters.",
      target_category_, SLOT(updateForceArrowGeometry()), this);

  // Center of Mass trajectory properties
  com_enable_property_ =
      new BoolProperty("Enable", true, "Enable/disable the CoM display",
                       com_category_, SLOT(updateCoMEnable()), this);
  com_style_property_ =
      new EnumProperty("Line Style", "Billboards",
                       "The rendering operation to use to draw the grid lines.",
                       com_category_, SLOT(updateCoMStyle()), this);
  com_style_property_->addOption("Billboards", BILLBOARDS);
  com_style_property_->addOption("Lines", LINES);
  com_style_property_->addOption("Points", POINTS);
  com_line_width_property_ =
      new FloatProperty("Line Width", 0.01,
                        "The width, in meters, of each path line. "
                        "Only works with the 'Billboards' and 'Points' style.",
                        com_category_, SLOT(updateCoMLineProperties()), this);
  com_line_width_property_->setMin(0.001);
  com_line_width_property_->show();
  com_color_property_ = new ColorProperty(
      "Line Color", QColor(0, 85, 255), "Color to draw the path.",
      com_category_, SLOT(updateCoMLineProperties()), this);
  com_scale_property_ = new FloatProperty(
      "Axes Scale", 1.0, "The scale of the axes that describe the orientation.",
      com_category_, SLOT(updateCoMLineProperties()), this);
  com_alpha_property_ = new FloatProperty(
      "Alpha", 1.0, "Amount of transparency to apply to the trajectory.",
      com_category_, SLOT(updateCoMLineProperties()), this);
  com_alpha_property_->setMin(0);
  com_alpha_property_->setMax(1);

  // End-effector trajectory properties
  contact_enable_property_ =
      new BoolProperty("Enable", true, "Enable/disable the Contact display",
                       contact_category_, SLOT(updateContactEnable()), this);
  contact_style_property_ =
      new EnumProperty("Line Style", "Billboards",
                       "The rendering operation to use to draw the grid lines.",
                       contact_category_, SLOT(updateContactStyle()), this);
  contact_style_property_->addOption("Billboards", BILLBOARDS);
  contact_style_property_->addOption("Lines", LINES);
  contact_style_property_->addOption("Points", POINTS);
  contact_line_width_property_ = new FloatProperty(
      "Line Width", 0.01,
      "The width, in meters, of each trajectory line. "
      "Only works with the 'Billboards' and 'Points' style.",
      contact_category_, SLOT(updateContactLineProperties()), this);
  contact_line_width_property_->setMin(0.001);
  contact_line_width_property_->show();
  contact_color_property_ = new ColorProperty(
      "Line Color", QColor(255, 0, 127), "Color to draw the trajectory.",
      contact_category_, SLOT(updateContactLineProperties()), this);
  contact_scale_property_ = new FloatProperty(
      "Axes Scale", 1.0, "The scale of the axes that describe the orientation.",
      contact_category_, SLOT(updateContactLineProperties()), this);
  contact_alpha_property_ = new FloatProperty(
      "Alpha", 1.0, "Amount of transparency to apply to the trajectory.",
      contact_category_, SLOT(updateContactLineProperties()), this);
  contact_alpha_property_->setMin(0);
  contact_alpha_property_->setMax(1);
}

WholeBodyTrajectoryDisplay::~WholeBodyTrajectoryDisplay() {
  clearRobotModel();
  destroyObjects();
}

void WholeBodyTrajectoryDisplay::onInitialize() {
  MFDClass::onInitialize();
  robot_.reset(new rviz::Robot(scene_node_, context_,
                               "Robot: " + getName().toStdString(), this));
  updateRobotVisualVisible();
  updateRobotCollisionVisible();
  updateRobotAlpha();
}

void WholeBodyTrajectoryDisplay::onEnable() {
  loadRobotModel();
  updateTargetEnable();
  updateCoMEnable();
  updateContactEnable();
}

void WholeBodyTrajectoryDisplay::onDisable() {
  robot_->setVisible(false);
  clearRobotModel();
}

void WholeBodyTrajectoryDisplay::fixedFrameChanged() {
  if (is_info_) {
    // Visualization of the base trajectory
    processCoMTrajectory();
    // Visualization of the end-effector trajectory
    processContactTrajectory();
  }
}

void WholeBodyTrajectoryDisplay::reset() { MFDClass::reset(); }

void WholeBodyTrajectoryDisplay::updateCoMStyle() {
  LineStyle style = (LineStyle)com_style_property_->getOptionInt();
  switch (style) {
  case BILLBOARDS:
    com_line_width_property_->show();
    com_manual_object_.reset();
    com_points_.clear();
    break;
  case LINES:
    com_line_width_property_->hide();
    com_billboard_line_.reset();
    com_points_.clear();
    break;
  case POINTS:
    com_line_width_property_->show();
    com_manual_object_.reset();
    com_billboard_line_.reset();
    break;
  }
  if (is_info_) {
    processCoMTrajectory();
  }
}

void WholeBodyTrajectoryDisplay::updateRobotDescription() {
  if (isEnabled())
    loadRobotModel();
}

void WholeBodyTrajectoryDisplay::updateRobotVisualVisible() {
  robot_->setVisualVisible(robot_visual_enabled_property_->getValue().toBool());
  context_->queueRender();
}

void WholeBodyTrajectoryDisplay::updateRobotCollisionVisible() {
  robot_->setCollisionVisible(
      robot_collision_enabled_property_->getValue().toBool());
  context_->queueRender();
}

void WholeBodyTrajectoryDisplay::updateRobotAlpha() {
  robot_->setAlpha(robot_alpha_property_->getFloat());
  context_->queueRender();
}

void WholeBodyTrajectoryDisplay::updateForceColorAndAlpha() {
  Ogre::ColourValue color = force_color_property_->getOgreColor();
  color.a = force_alpha_property_->getFloat();
  for (size_t i = 0; i < force_visual_.size(); ++i) {
    force_visual_[i]->setColor(color.r, color.g, color.b, color.a);
  }
  context_->queueRender();
}

void WholeBodyTrajectoryDisplay::updateForceArrowGeometry() {
  const float &shaft_length = force_shaft_length_property_->getFloat();
  const float &shaft_radius = force_shaft_radius_property_->getFloat();
  const float &head_length = force_head_length_property_->getFloat();
  const float &head_radius = force_head_radius_property_->getFloat();
  for (size_t i = 0; i < force_visual_.size(); ++i) {
    force_visual_[i]->setProperties(shaft_length, shaft_radius, head_length,
                                    head_radius);
  }
  context_->queueRender();
}

void WholeBodyTrajectoryDisplay::updateTargetEnable() {
  target_enable_ = target_enable_property_->getBool();
  if (target_enable_) {
    robot_->setVisible(true);
  } else {
    robot_->setVisible(false);
  }
}

void WholeBodyTrajectoryDisplay::updateCoMEnable() {
  com_enable_ = com_enable_property_->getBool();
  if (!com_enable_) {
    com_billboard_line_.reset();
    com_manual_object_.reset();
    com_points_.clear();
  }
  context_->queueRender();
}

void WholeBodyTrajectoryDisplay::updateContactEnable() {
  contact_enable_ = contact_enable_property_->getBool();
  if (!contact_enable_) {
    for (std::size_t i = 0; i < contact_manual_object_.size(); ++i) {
      contact_manual_object_[i].reset();
    }
    for (std::size_t i = 0; i < contact_billboard_line_.size(); ++i) {
      contact_billboard_line_[i].reset();
    }
    for (std::size_t i = 0; i < contact_points_.size(); ++i) {
      std::size_t n_elems = contact_points_[i].size();
      for (std::size_t j = 0; j < n_elems; ++j)
        contact_points_[i][j].reset();
      contact_points_[i].clear();
    }
  }
  context_->queueRender();
}

void WholeBodyTrajectoryDisplay::updateCoMLineProperties() {
  LineStyle style = (LineStyle)com_style_property_->getOptionInt();
  float line_width = com_line_width_property_->getFloat();
  float scale = com_scale_property_->getFloat();
  com_axes_enable_ = true;
  if (scale == 0) {
    com_axes_enable_ = false;
    com_axes_.clear();
  }
  Ogre::ColourValue color = com_color_property_->getOgreColor();
  color.a = com_alpha_property_->getFloat();
  if (style == BILLBOARDS) {
    com_billboard_line_->setLineWidth(line_width);
    com_billboard_line_->setColor(color.r, color.g, color.b, color.a);
    if (com_axes_enable_) {
      std::size_t num_axes = com_axes_.size();
      for (std::size_t i = 0; i < num_axes; ++i) {
        Ogre::ColourValue x_color = com_axes_[i]->getDefaultXColor();
        Ogre::ColourValue y_color = com_axes_[i]->getDefaultYColor();
        Ogre::ColourValue z_color = com_axes_[i]->getDefaultZColor();
        x_color.a = com_alpha_property_->getFloat();
        y_color.a = com_alpha_property_->getFloat();
        z_color.a = com_alpha_property_->getFloat();
        com_axes_[i]->setXColor(x_color);
        com_axes_[i]->setYColor(y_color);
        com_axes_[i]->setZColor(z_color);
        com_axes_[i]->getSceneNode()->setVisible(true);
        com_axes_[i]->setScale(Ogre::Vector3(scale, scale, scale));
      }
    }
  } else if (style == LINES) {
    // we have to process again the base trajectory
    if (is_info_)
      processCoMTrajectory();
  } else {
    std::size_t n_points = com_points_.size();
    for (std::size_t i = 0; i < n_points; ++i) {
      com_points_[i]->setColor(color.r, color.g, color.b, color.a);
      com_points_[i]->setRadius(line_width);
    }
    if (com_axes_enable_) {
      std::size_t num_axes = com_axes_.size();
      for (std::size_t i = 0; i < num_axes; ++i) {
        Ogre::ColourValue x_color = com_axes_[i]->getDefaultXColor();
        Ogre::ColourValue y_color = com_axes_[i]->getDefaultYColor();
        Ogre::ColourValue z_color = com_axes_[i]->getDefaultZColor();
        x_color.a = com_alpha_property_->getFloat();
        y_color.a = com_alpha_property_->getFloat();
        z_color.a = com_alpha_property_->getFloat();
        com_axes_[i]->setXColor(x_color);
        com_axes_[i]->setYColor(y_color);
        com_axes_[i]->setZColor(z_color);
        com_axes_[i]->getSceneNode()->setVisible(true);
        com_axes_[i]->setScale(Ogre::Vector3(scale, scale, scale));
      }
    }
  }
  context_->queueRender();
}

void WholeBodyTrajectoryDisplay::updateContactStyle() {
  LineStyle style = (LineStyle)contact_style_property_->getOptionInt();
  std::size_t n_contacts = contact_billboard_line_.size();
  std::size_t n_points = contact_points_.size();
  switch (style) {
  case BILLBOARDS:
    contact_line_width_property_->show();
    for (std::size_t i = 0; i < n_contacts; ++i) {
      contact_manual_object_[i].reset();
    }
    for (std::size_t i = 0; i < n_points; ++i) {
      std::size_t n_elems = contact_points_[i].size();
      for (std::size_t j = 0; j < n_elems; ++j)
        contact_points_[i][j].reset();
      contact_points_[i].clear();
    }
    break;
  case LINES:
    contact_line_width_property_->hide();
    for (std::size_t i = 0; i < n_contacts; ++i) {
      contact_billboard_line_[i].reset();
    }
    for (std::size_t i = 0; i < n_points; ++i) {
      std::size_t n_elems = contact_points_[i].size();
      for (std::size_t j = 0; j < n_elems; ++j)
        contact_points_[i][j].reset();
      contact_points_[i].clear();
    }
    break;
  case POINTS:
    contact_line_width_property_->show();
    for (std::size_t i = 0; i < n_contacts; ++i) {
      contact_manual_object_[i].reset();
      contact_billboard_line_[i].reset();
    }
  }
  if (is_info_) {
    processContactTrajectory();
  }
}

void WholeBodyTrajectoryDisplay::updateContactLineProperties() {
  LineStyle style = (LineStyle)contact_style_property_->getOptionInt();
  float line_width = contact_line_width_property_->getFloat();
  Ogre::ColourValue color = contact_color_property_->getOgreColor();
  float scale = contact_scale_property_->getFloat();
  contact_axes_enable_ = true;
  if (scale == 0) {
    contact_axes_enable_ = false;
    contact_axes_.clear();
  }
  color.a = contact_alpha_property_->getFloat();
  if (style == BILLBOARDS) {
    std::size_t n_contacts = contact_billboard_line_.size();
    for (std::size_t i = 0; i < n_contacts; ++i) {
      contact_billboard_line_[i]->setLineWidth(line_width);
      contact_billboard_line_[i]->setColor(color.r, color.g, color.b, color.a);
    }
    if (contact_axes_enable_) {
      std::size_t num_axes = contact_axes_.size();
      for (std::size_t i = 0; i < num_axes; ++i) {
        Ogre::ColourValue x_color = contact_axes_[i]->getDefaultXColor();
        Ogre::ColourValue y_color = contact_axes_[i]->getDefaultYColor();
        Ogre::ColourValue z_color = contact_axes_[i]->getDefaultZColor();
        x_color.a = contact_alpha_property_->getFloat();
        y_color.a = contact_alpha_property_->getFloat();
        z_color.a = contact_alpha_property_->getFloat();
        contact_axes_[i]->setXColor(x_color);
        contact_axes_[i]->setYColor(y_color);
        contact_axes_[i]->setZColor(z_color);
        contact_axes_[i]->getSceneNode()->setVisible(true);
        contact_axes_[i]->setScale(Ogre::Vector3(scale, scale, scale));
      }
    }
  } else if (style == LINES) {
    // we have to process again the contact trajectory
    if (is_info_)
      processContactTrajectory();
  } else {
    std::size_t n_points = contact_points_.size();
    for (std::size_t i = 0; i < n_points; ++i) {
      std::size_t n_contacts = contact_points_[i].size();
      for (std::size_t j = 0; j < n_contacts; ++j) {
        contact_points_[i][j]->setColor(color.r, color.g, color.b, color.a);
        contact_points_[i][j]->setRadius(line_width);
      }
    }
    if (contact_axes_enable_) {
      std::size_t num_axes = contact_axes_.size();
      for (std::size_t i = 0; i < num_axes; ++i) {
        Ogre::ColourValue x_color = contact_axes_[i]->getDefaultXColor();
        Ogre::ColourValue y_color = contact_axes_[i]->getDefaultYColor();
        Ogre::ColourValue z_color = contact_axes_[i]->getDefaultZColor();
        x_color.a = contact_alpha_property_->getFloat();
        y_color.a = contact_alpha_property_->getFloat();
        z_color.a = contact_alpha_property_->getFloat();
        contact_axes_[i]->setXColor(x_color);
        contact_axes_[i]->setYColor(y_color);
        contact_axes_[i]->setZColor(z_color);
        contact_axes_[i]->getSceneNode()->setVisible(true);
        contact_axes_[i]->setScale(Ogre::Vector3(scale, scale, scale));
      }
    }
  }
  context_->queueRender();
}

void WholeBodyTrajectoryDisplay::processMessage(
    const whole_body_state_msgs::WholeBodyTrajectory::ConstPtr &msg) {
  // Updating the message
  msg_ = msg;
  is_info_ = true;
  // Destroy all the old elements
  destroyObjects();
  // Visualization of the base trajectory
  processTargetPosture();
  // Visualization of the base trajectory
  processCoMTrajectory();
  // Visualization of the end-effector trajectory
  processContactTrajectory();
}

void WholeBodyTrajectoryDisplay::processTargetPosture() {
  if (target_enable_) {
    Ogre::Quaternion orientation;
    Ogre::Vector3 position;
    if (!context_->getFrameManager()->getTransform(
            msg_->header.frame_id, msg_->header.stamp, position, orientation)) {
      ROS_DEBUG("Error transforming from frame '%s' to frame '%s'",
                msg_->header.frame_id.c_str(), qPrintable(fixed_frame_));
      return;
    }

    const whole_body_state_msgs::WholeBodyState &state =
        msg_->trajectory.back();
    Eigen::VectorXd q = Eigen::VectorXd::Zero(model_.nq);
    q(3) = state.centroidal.base_orientation.x;
    q(4) = state.centroidal.base_orientation.y;
    q(5) = state.centroidal.base_orientation.z;
    q(6) = state.centroidal.base_orientation.w;
    std::size_t n_joints = state.joints.size();
    for (std::size_t j = 0; j < n_joints; ++j) {
      pinocchio::JointIndex jointId =
          model_.getJointId(state.joints[j].name) - 2;
      q(jointId + 7) = state.joints[j].position;
    }
    pinocchio::centerOfMass(model_, data_, q);
    q(0) = state.centroidal.com_position.x - data_.com[0](0);
    q(1) = state.centroidal.com_position.y - data_.com[0](1);
    q(2) = state.centroidal.com_position.z - data_.com[0](2);
    robot_->update(PinocchioLinkUpdater(
        model_, data_, q,
        boost::bind(linkUpdaterStatusFunction, _1, _2, _3, this)));

    size_t n_contacts = state.contacts.size();
    force_visual_.clear();
    for (size_t i = 0; i < n_contacts; ++i) {
      const whole_body_state_msgs::ContactState &contact = state.contacts[i];
      // Getting the contact position
      Ogre::Vector3 contact_pos(contact.pose.position.x,
                                contact.pose.position.y,
                                contact.pose.position.z);
      // Getting the force direction
      Eigen::Vector3d for_ref_dir = -Eigen::Vector3d::UnitZ();
      Eigen::Vector3d for_dir(contact.wrench.force.x, contact.wrench.force.y,
                              contact.wrench.force.z);
      if (for_dir.norm() > 0. && std::isfinite(contact_pos.x) &&
          std::isfinite(contact_pos.y) && std::isfinite(contact_pos.z)) {
        Eigen::Quaterniond for_q;
        for_q.setFromTwoVectors(for_ref_dir, for_dir);
        Ogre::Quaternion contact_for_orientation(for_q.w(), for_q.x(),
                                                 for_q.y(), for_q.z());
        // We are keeping a vector of visual pointers. This creates the next
        // one and stores it in the vector
        boost::shared_ptr<ArrowVisual> arrow;
        arrow.reset(new ArrowVisual(context_->getSceneManager(), scene_node_));
        arrow->setArrow(contact_pos, contact_for_orientation);
        arrow->setFramePosition(position);
        arrow->setFrameOrientation(orientation);
        // Setting the arrow color and properties
        Ogre::ColourValue color = force_color_property_->getOgreColor();
        color.a = force_alpha_property_->getFloat();
        arrow->setColor(color.r, color.g, color.b, color.a);
        const float &shaft_length =
            force_shaft_length_property_->getFloat() * for_dir.norm() / weight_;
        const float &shaft_radius = force_shaft_radius_property_->getFloat();
        const float &head_length = force_head_length_property_->getFloat();
        const float &head_radius = force_head_radius_property_->getFloat();
        arrow->setProperties(shaft_length, shaft_radius, head_length,
                             head_radius);
        // And send it to the end of the vector
        if (std::isfinite(shaft_length) && std::isfinite(shaft_radius) &&
            std::isfinite(head_length) && std::isfinite(head_radius)) {
          force_visual_.push_back(arrow);
        }
      }
    }
  }
}

void WholeBodyTrajectoryDisplay::processCoMTrajectory() {
  // Lookup transform into fixed frame
  if (com_enable_) {
    Ogre::Vector3 position;
    Ogre::Quaternion orientation;
    if (!context_->getFrameManager()->getTransform(msg_->header, position,
                                                   orientation)) {
      ROS_DEBUG("Error transforming from frame '%s' to frame '%s'",
                msg_->header.frame_id.c_str(), qPrintable(fixed_frame_));
    }
    Ogre::Matrix4 transform(orientation);
    transform.setTrans(position);

    // Visualization of the base trajectory
    // Getting the base trajectory style
    LineStyle base_style = (LineStyle)com_style_property_->getOptionInt();

    // Getting the base trajectory color
    Ogre::ColourValue base_color = com_color_property_->getOgreColor();
    base_color.a = com_alpha_property_->getFloat();

    // Visualization of the base trajectory
    std::size_t n_points = msg_->trajectory.size();
    com_axes_.clear();
    for (std::size_t i = 0; i < n_points; ++i) {
      const whole_body_state_msgs::WholeBodyState &state = msg_->trajectory[i];
      // Obtaining the CoM position and the base orientation
      Ogre::Vector3 com_position;
      Ogre::Quaternion base_orientation;
      com_position.x = state.centroidal.com_position.x;
      com_position.y = state.centroidal.com_position.y;
      com_position.z = state.centroidal.com_position.z;
      base_orientation.x = state.centroidal.base_orientation.x;
      base_orientation.y = state.centroidal.base_orientation.y;
      base_orientation.z = state.centroidal.base_orientation.z;
      base_orientation.w = state.centroidal.base_orientation.w;
      // sanity checks
      if (!(std::isfinite(com_position.x) && std::isfinite(com_position.y) &&
            std::isfinite(com_position.z))) {
        std::cerr << "CoM position is not finite, resetting to zero"
                  << std::endl;
        com_position.x = 0.0;
        com_position.y = 0.0;
        com_position.z = 0.0;
      }
      if (!(std::isfinite(base_orientation.x) &&
            std::isfinite(base_orientation.y) &&
            std::isfinite(base_orientation.z) &&
            std::isfinite(base_orientation.w))) {
        std::cerr << "Body orientation is not finite, resetting to [0 0 0 1]"
                  << std::endl;
        base_orientation.x = 0.;
        base_orientation.y = 0.;
        base_orientation.z = 0.;
        base_orientation.w = 1.;
      }

      Ogre::Vector3 point_position = transform * com_position;
      if (com_axes_enable_) {
        pushBackCoMAxes(point_position, base_orientation * orientation);
      }
      switch (base_style) {
      case BILLBOARDS: {
        // Getting the base line width
        if (i == 0) {
          float base_line_width = com_line_width_property_->getFloat();
          com_billboard_line_.reset(
              new rviz::BillboardLine(scene_manager_, scene_node_));
          com_billboard_line_->setNumLines(1);
          com_billboard_line_->setMaxPointsPerLine(n_points);
          com_billboard_line_->setLineWidth(base_line_width);
        }
        com_billboard_line_->addPoint(point_position, base_color);
      } break;
      case LINES: {
        if (i == 0) {
          com_manual_object_.reset(scene_manager_->createManualObject());
          com_manual_object_->setDynamic(true);
          scene_node_->attachObject(com_manual_object_.get());
          com_manual_object_->estimateVertexCount(n_points);
          com_manual_object_->begin("BaseWhiteNoLighting",
                                    Ogre::RenderOperation::OT_LINE_STRIP);
        }
        com_manual_object_->position(point_position.x, point_position.y,
                                     point_position.z);
        com_manual_object_->colour(base_color);
      } break;
      case POINTS: {
        if (i == 0) {
          com_points_.clear();
        }
        float base_line_width = com_line_width_property_->getFloat();
        // We are keeping a vector of CoM visual pointers. This creates the next
        // one and stores it in the vector
        boost::shared_ptr<PointVisual> point_visual;
        point_visual.reset(
            new PointVisual(context_->getSceneManager(), scene_node_));
        point_visual->setColor(base_color.r, base_color.g, base_color.b,
                               base_color.a);
        point_visual->setRadius(base_line_width);
        point_visual->setPoint(com_position);
        point_visual->setFramePosition(position);
        point_visual->setFrameOrientation(orientation);
        // And send it to the end of the vector
        com_points_.push_back(point_visual);
      } break;
      }
    }
    if (base_style == LINES) {
      com_manual_object_->end();
    }
  }
}

void WholeBodyTrajectoryDisplay::processContactTrajectory() {
  if (contact_enable_) {
    // Lookup transform into fixed frame
    Ogre::Vector3 position;
    Ogre::Quaternion orientation;
    if (!context_->getFrameManager()->getTransform(msg_->header, position,
                                                   orientation)) {
      ROS_DEBUG("Error transforming from frame '%s' to frame '%s'",
                msg_->header.frame_id.c_str(), qPrintable(fixed_frame_));
    }
    Ogre::Matrix4 transform(orientation);
    transform.setTrans(position);

    // Visualization of the end-effector trajectory
    // Getting the end-effector trajectory style
    uint32_t n_points = msg_->trajectory.size();
    LineStyle contact_style =
        (LineStyle)contact_style_property_->getOptionInt();

    // Getting the end-effector trajectory color
    Ogre::ColourValue contact_color = contact_color_property_->getOgreColor();
    contact_color.a = contact_alpha_property_->getFloat();

    // Getting the number of contact trajectories
    std::size_t n_traj = 0;
    std::map<std::string, std::size_t> contact_traj_id;
    for (std::size_t i = 0; i < n_points; ++i) {
      const whole_body_state_msgs::WholeBodyState &state = msg_->trajectory[i];
      std::size_t n_contacts = state.contacts.size();
      for (std::size_t k = 0; k < n_contacts; ++k) {
        whole_body_state_msgs::ContactState contact =
            msg_->trajectory[i].contacts[k];
        if (contact_traj_id.find(contact.name) ==
            contact_traj_id.end()) { // a new swing trajectory
          contact_traj_id[contact.name] = n_traj;
          // Incrementing the counter (id) of swing trajectories
          ++n_traj;
        }
      }
    }

    // Visualizing the different end-effector trajectories
    contact_traj_id.clear();
    contact_axes_.clear();
    std::map<std::size_t, std::size_t> contact_vec_id;
    float contact_line_width = contact_line_width_property_->getFloat();
    switch (contact_style) {
    case BILLBOARDS: {
      // Getting the end-effector line width
      contact_billboard_line_.clear();
      contact_billboard_line_.resize(n_traj);
    } break;
    case LINES: {
      contact_manual_object_.clear();
      contact_manual_object_.resize(n_traj);
    } break;
    case POINTS: {
      // Getting the end-effector line width
      contact_points_.clear();
      contact_points_.resize(n_points);
    } break;
    }

    std::size_t traj_id = 0;
    for (std::size_t i = 0; i < n_points; ++i) {
      const whole_body_state_msgs::WholeBodyState &state = msg_->trajectory[i];
      std::size_t n_contacts = state.contacts.size();
      for (std::size_t k = 0; k < n_contacts; ++k) {
        const whole_body_state_msgs::ContactState &contact = state.contacts[k];
        if (contact_traj_id.find(contact.name) ==
            contact_traj_id.end()) { // a new swing trajectory
          contact_traj_id[contact.name] = traj_id;
          contact_vec_id[traj_id] = k;
          switch (contact_style) {
          case BILLBOARDS: {
            contact_billboard_line_[traj_id].reset(
                new rviz::BillboardLine(scene_manager_, scene_node_));
            contact_billboard_line_[traj_id]->setNumLines(1);
            contact_billboard_line_[traj_id]->setMaxPointsPerLine(n_points);
            contact_billboard_line_[traj_id]->setLineWidth(contact_line_width);
          } break;
          case LINES: {
            contact_manual_object_[traj_id].reset(
                scene_manager_->createManualObject());
            contact_manual_object_[traj_id]->setDynamic(true);
            scene_node_->attachObject(contact_manual_object_[traj_id].get());
            contact_manual_object_[traj_id]->estimateVertexCount(n_points);
            contact_manual_object_[traj_id]->begin(
                "BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP);
          } break;
          case POINTS: {
            // do nothing
          } break;
          }
          // Incrementing the counter (id) of swing trajectories
          ++traj_id;
        } else {
          std::size_t swing_idx = contact_traj_id.find(contact.name)->second;
          if (k != contact_vec_id.find(swing_idx)
                       ->second) { // change the vector index
            contact_vec_id[swing_idx] = k;
          }
        }
      }
      // Adding the contact points for the current swing trajectories
      std::size_t contact_idx = 0;
      if (contact_style == POINTS) {
        // Updating the size
        contact_points_[i].clear();
        contact_points_[i].resize(n_contacts);
      }
      for (std::map<std::string, std::size_t>::iterator traj_it =
               contact_traj_id.begin();
           traj_it != contact_traj_id.end(); ++traj_it) {
        std::size_t traj_id = traj_it->second;
        std::size_t id = contact_vec_id.find(traj_id)->second;
        if (id < n_contacts) {
          const whole_body_state_msgs::ContactState &contact =
              state.contacts[id];
          Ogre::Vector3 contact_position;
          Ogre::Quaternion contact_orientation;
          contact_position.x = contact.pose.position.x;
          contact_position.y = contact.pose.position.y;
          contact_position.z = contact.pose.position.z;
          contact_orientation.x = contact.pose.orientation.x;
          contact_orientation.y = contact.pose.orientation.y;
          contact_orientation.z = contact.pose.orientation.z;
          contact_orientation.w = contact.pose.orientation.w;
          // sanity check orientation
          if (!(std::isfinite(contact_position.x) &&
                std::isfinite(contact_position.y) &&
                std::isfinite(contact_position.z))) {
            std::cerr << "Contact trajectory is not finite, resetting to zero!"
                      << std::endl;
            contact_position.x = 0.0;
            contact_position.y = 0.0;
            contact_position.z = 0.0;
          }
          if (!(std::isfinite(contact_orientation.x) &&
                std::isfinite(contact_orientation.y) &&
                std::isfinite(contact_orientation.z) &&
                std::isfinite(contact_orientation.w))) {
            std::cerr
                << "Contact orientation is not finite, resetting to [0 0 0 1]"
                << std::endl;
            contact_orientation.x = 0.;
            contact_orientation.y = 0.;
            contact_orientation.z = 0.;
            contact_orientation.w = 1.;
          }
          Ogre::Vector3 point_position = transform * contact_position;
          if (contact_axes_enable_) {
            pushBackContactAxes(point_position,
                                contact_orientation * orientation);
          }
          switch (contact_style) {
          case BILLBOARDS: {
            contact_billboard_line_[traj_id]->addPoint(point_position,
                                                       contact_color);
          } break;
          case LINES: {
            contact_manual_object_[traj_id]->position(
                point_position.x, point_position.y, point_position.z);
            contact_manual_object_[traj_id]->colour(contact_color);
          } break;
          case POINTS: {
            contact_points_[i][contact_idx].reset(
                new PointVisual(context_->getSceneManager(), scene_node_));
            contact_points_[i][contact_idx]->setColor(
                contact_color.r, contact_color.g, contact_color.b,
                contact_color.a);
            contact_points_[i][contact_idx]->setRadius(contact_line_width);
            contact_points_[i][contact_idx]->setPoint(point_position);
            contact_points_[i][contact_idx]->setFramePosition(position);
            contact_points_[i][contact_idx]->setFrameOrientation(orientation);
            ++contact_idx;
          } break;
          }
        }
      }
    }

    // Ending the contact manual objects
    if (contact_style == LINES) {
      for (std::size_t i = 0; i < traj_id; ++i) {
        contact_manual_object_[i]->end();
      }
    }
  }
}

void WholeBodyTrajectoryDisplay::loadRobotModel() {
  clearStatuses();
  context_->queueRender();
  std::string content;
  if (!update_nh_.getParam(robot_description_property_->getStdString(),
                           content)) {
    std::string loc;
    if (update_nh_.searchParam(robot_description_property_->getStdString(),
                               loc)) {
      update_nh_.getParam(loc, content);
    } else {
      clearRobotModel();
      setStatus(StatusProperty::Error, "URDF",
                "Parameter [" + robot_description_property_->getString() +
                    "] does not exist, and was not found by searchParam()");
      // try again in a second
      QTimer::singleShot(1000, this, SLOT(updateRobotDescription()));
      return;
    }
  }
  if (content.empty()) {
    clearRobotModel();
    setStatus(StatusProperty::Error, "URDF", "URDF is empty");
    return;
  }
  if (content == robot_description_) {
    return;
  }
  robot_description_ = content;
  urdf::Model descr;
  if (!descr.initString(robot_description_)) {
    clearRobotModel();
    setStatus(StatusProperty::Error, "URDF", "Failed to parse URDF model");
    return;
  }
  pinocchio::urdf::buildModelFromXML(robot_description_,
                                     pinocchio::JointModelFreeFlyer(), model_);
  data_ = pinocchio::Data(model_);
  double gravity = model_.gravity.linear().norm();
  weight_ = pinocchio::computeTotalMass(model_) * gravity;
  robot_->load(descr);
  updateTargetEnable();
  setStatus(StatusProperty::Ok, "URDF", "URDF parsed OK");
}

void WholeBodyTrajectoryDisplay::clearRobotModel() {
  robot_->clear();
  clearStatuses();
  robot_description_.clear();
  model_ = pinocchio::Model();
  data_ = pinocchio::Data();
}

void WholeBodyTrajectoryDisplay::destroyObjects() {
  com_manual_object_.reset();
  com_billboard_line_.reset();
  com_points_.clear();
  com_axes_.clear();
  contact_manual_object_.clear();
  contact_billboard_line_.clear();
  for (std::size_t i = 0; i < contact_points_.size(); ++i) {
    contact_points_[i].clear();
  }
  contact_points_.clear();
  contact_axes_.clear();
}

void WholeBodyTrajectoryDisplay::pushBackCoMAxes(
    const Ogre::Vector3 &axes_position,
    const Ogre::Quaternion &axes_orientation) {
  // We are keeping a vector of CoM frame pointers. This creates the next
  // one and stores it in the vector
  float scale = com_scale_property_->getFloat();
  // Adding the frame with a distant from the last one
  float sq_distant = axes_position.squaredDistance(last_point_position_);
  if (sq_distant >= scale * scale * 0.0032) {
    boost::shared_ptr<rviz::Axes> axes;
    axes.reset(new Axes(scene_manager_, scene_node_, 0.04, 0.008));
    axes->setPosition(axes_position);
    axes->setOrientation(axes_orientation);
    Ogre::ColourValue x_color = axes->getDefaultXColor();
    Ogre::ColourValue y_color = axes->getDefaultYColor();
    Ogre::ColourValue z_color = axes->getDefaultZColor();
    x_color.a = com_alpha_property_->getFloat();
    y_color.a = com_alpha_property_->getFloat();
    z_color.a = com_alpha_property_->getFloat();
    axes->setXColor(x_color);
    axes->setYColor(y_color);
    axes->setZColor(z_color);
    axes->getSceneNode()->setVisible(true);
    axes->setScale(Ogre::Vector3(scale, scale, scale));
    com_axes_.push_back(axes);
    last_point_position_ = axes_position;
  }
}

void WholeBodyTrajectoryDisplay::pushBackContactAxes(
    const Ogre::Vector3 &axes_position,
    const Ogre::Quaternion &axes_orientation) {
  // We are keeping a vector of contact frame pointers. This creates the next
  // one and stores it in the vector
  float scale = contact_scale_property_->getFloat();
  // Adding the frame with a distant from the last one
  float sq_distant = axes_position.squaredDistance(last_point_position_);
  if (sq_distant >= scale * scale * 0.0032) {
    boost::shared_ptr<rviz::Axes> axes;
    axes.reset(new Axes(scene_manager_, scene_node_, 0.04, 0.008));
    axes->setPosition(axes_position);
    axes->setOrientation(axes_orientation);
    Ogre::ColourValue x_color = axes->getDefaultXColor();
    Ogre::ColourValue y_color = axes->getDefaultYColor();
    Ogre::ColourValue z_color = axes->getDefaultZColor();
    x_color.a = com_alpha_property_->getFloat();
    y_color.a = com_alpha_property_->getFloat();
    z_color.a = com_alpha_property_->getFloat();
    axes->setXColor(x_color);
    axes->setYColor(y_color);
    axes->setZColor(z_color);
    axes->getSceneNode()->setVisible(true);
    axes->setScale(Ogre::Vector3(scale, scale, scale));
    contact_axes_.push_back(axes);
    last_point_position_ = axes_position;
  }
}

} // namespace whole_body_state_rviz_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(whole_body_state_rviz_plugin::WholeBodyTrajectoryDisplay,
                       rviz::Display)
