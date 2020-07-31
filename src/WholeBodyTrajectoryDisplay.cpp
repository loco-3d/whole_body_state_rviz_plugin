///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2020, University of Edinburgh, Istituto Italiano di Tecnologia
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "state_rviz_plugin/WholeBodyTrajectoryDisplay.h"
#include <Eigen/Dense>
#include <OgreBillboardSet.h>
#include <OgreManualObject.h>
#include <OgreMatrix4.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <boost/bind.hpp>
#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/ogre_helpers/axes.h>
#include <rviz/ogre_helpers/billboard_line.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/vector_property.h>
#include <rviz/validate_floats.h>
#include <tf/transform_listener.h>

using namespace rviz;

namespace state_rviz_plugin {

WholeBodyTrajectoryDisplay::WholeBodyTrajectoryDisplay()
    : is_info_(false), com_enable_(true) {
  // Category Groups
  com_category_ = new rviz::Property("Center of Mass", QVariant(), "", this);
  contact_category_ = new rviz::Property("End-Effector", QVariant(), "", this);

  // Base trajectory properties
  com_enable_property_ =
      new BoolProperty("Enable", true, "Enable/disable the CoM display",
                       com_category_, SLOT(updateCoMEnable()), this);
  com_style_property_ =
      new EnumProperty("Line Style", "Points",
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
  contact_style_property_ =
      new EnumProperty("Line Style", "Points",
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
      "Line Color", QColor(0, 85, 255), "Color to draw the trajectory.",
      contact_category_, SLOT(updateContactLineProperties()), this);

  contact_alpha_property_ = new FloatProperty(
      "Alpha", 1.0, "Amount of transparency to apply to the trajectory.",
      contact_category_, SLOT(updateContactLineProperties()), this);
  contact_alpha_property_->setMin(0);
  contact_alpha_property_->setMax(1);
}

WholeBodyTrajectoryDisplay::~WholeBodyTrajectoryDisplay() { destroyObjects(); }

void WholeBodyTrajectoryDisplay::onInitialize() { MFDClass::onInitialize(); }

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
  case LINES:
    com_line_width_property_->hide();
    com_billboard_line_.reset();
    com_points_.clear();
    break;
  case BILLBOARDS:
    com_line_width_property_->show();
    com_manual_object_.reset();
    com_points_.clear();
    break;
  case POINTS:
    com_line_width_property_->show();
    com_manual_object_.reset();
    com_billboard_line_.reset();
    break;
  }

  if (is_info_)
    processCoMTrajectory();
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

void WholeBodyTrajectoryDisplay::updateCoMLineProperties() {
  LineStyle style = (LineStyle)com_style_property_->getOptionInt();
  float line_width = com_line_width_property_->getFloat();
  float scale = com_scale_property_->getFloat();
  Ogre::ColourValue color = com_color_property_->getOgreColor();
  color.a = com_alpha_property_->getFloat();

  if (style == BILLBOARDS) {
    com_billboard_line_->setLineWidth(line_width);
    com_billboard_line_->setColor(color.r, color.g, color.b, color.a);
    uint32_t num_axes = com_axes_.size();
    for (uint32_t i = 0; i < num_axes; i++) {
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
  } else if (style == LINES) {
    // we have to process again the base trajectory
    if (is_info_)
      processCoMTrajectory();
  } else {
    uint32_t num_points = com_points_.size();
    for (uint32_t i = 0; i < num_points; ++i) {
      com_points_[i]->setColor(color.r, color.g, color.b, color.a);
      com_points_[i]->setRadius(line_width);
    }

    uint32_t num_axes = com_axes_.size();
    for (uint32_t i = 0; i < num_axes; i++) {
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

  context_->queueRender();
}

void WholeBodyTrajectoryDisplay::updateContactStyle() {
  LineStyle style = (LineStyle)contact_style_property_->getOptionInt();
  uint32_t num_contact = contact_billboard_line_.size();
  uint32_t num_points = contact_points_.size();

  switch (style) {
  case LINES:
    contact_line_width_property_->hide();
    for (uint32_t i = 0; i < num_contact; i++) {
      contact_billboard_line_[i].reset();
    }
    for (uint32_t i = 0; i < num_points; i++) {
      uint32_t num_elem = contact_points_[i].size();
      for (uint32_t j = 0; j < num_elem; j++)
        contact_points_[i][j].reset();
      contact_points_[i].clear();
    }
    break;
  case BILLBOARDS:
    contact_line_width_property_->show();
    for (uint32_t i = 0; i < num_contact; i++) {
      contact_manual_object_[i].reset();
    }
    for (uint32_t i = 0; i < num_points; i++) {
      uint32_t num_elem = contact_points_[i].size();
      for (uint32_t j = 0; j < num_elem; j++)
        contact_points_[i][j].reset();
      contact_points_[i].clear();
    }
    break;
  case POINTS:
    contact_line_width_property_->show();
    for (uint32_t i = 0; i < num_contact; i++) {
      contact_manual_object_[i].reset();
      contact_billboard_line_[i].reset();
    }
  }

  if (is_info_)
    processContactTrajectory();
}

void WholeBodyTrajectoryDisplay::updateContactLineProperties() {
  LineStyle style = (LineStyle)contact_style_property_->getOptionInt();
  float line_width = contact_line_width_property_->getFloat();
  Ogre::ColourValue color = contact_color_property_->getOgreColor();
  color.a = contact_alpha_property_->getFloat();

  if (style == BILLBOARDS) {
    uint32_t num_contact = contact_billboard_line_.size();
    for (uint32_t i = 0; i < num_contact; i++) {
      contact_billboard_line_[i]->setLineWidth(line_width);
      contact_billboard_line_[i]->setColor(color.r, color.g, color.b, color.a);
    }
  } else if (style == LINES) {
    // we have to process again the contact trajectory
    if (is_info_)
      processContactTrajectory();
  } else {
    uint32_t num_points = contact_points_.size();
    for (uint32_t i = 0; i < num_points; i++) {
      uint32_t num_contacts = contact_points_[i].size();
      for (uint32_t j = 0; j < num_contacts; j++) {
        contact_points_[i][j]->setColor(color.r, color.g, color.b, color.a);
        contact_points_[i][j]->setRadius(line_width);
      }
    }
  }

  context_->queueRender();
}

void WholeBodyTrajectoryDisplay::processMessage(
    const state_msgs::WholeBodyTrajectory::ConstPtr &msg) {
  // Updating the message
  msg_ = msg;
  is_info_ = true;

  // Destroy all the old elements
  destroyObjects();

  // Visualization of the base trajectory
  processCoMTrajectory();

  // Visualization of the end-effector trajectory
  processContactTrajectory();
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
    uint32_t num_points = msg_->trajectory.size();

    com_axes_.clear();
    for (uint32_t i = 0; i < num_points; ++i) {
      const state_msgs::WholeBodyState &state = msg_->trajectory[i];
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
      pushBackCoMAxes(point_position, base_orientation * orientation);

      switch (base_style) {
      case LINES: {
        if (i == 0) {
          com_manual_object_.reset(scene_manager_->createManualObject());
          com_manual_object_->setDynamic(true);
          scene_node_->attachObject(com_manual_object_.get());
          com_manual_object_->estimateVertexCount(num_points);
          com_manual_object_->begin("BaseWhiteNoLighting",
                                    Ogre::RenderOperation::OT_LINE_STRIP);
        }
        com_manual_object_->position(point_position.x, point_position.y,
                                     point_position.z);
        com_manual_object_->colour(base_color);
      } break;
      case BILLBOARDS: {
        // Getting the base line width
        if (i == 0) {
          float base_line_width = com_line_width_property_->getFloat();
          com_billboard_line_.reset(
              new rviz::BillboardLine(scene_manager_, scene_node_));
          com_billboard_line_->setNumLines(1);
          com_billboard_line_->setMaxPointsPerLine(num_points);
          com_billboard_line_->setLineWidth(base_line_width);
        }
        com_billboard_line_->addPoint(point_position, base_color);
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
  //   // Lookup transform into fixed frame
  //   Ogre::Vector3 position;
  //   Ogre::Quaternion orientation;
  //   if (!context_->getFrameManager()->getTransform(msg_->header, position,
  //                                                  orientation)) {
  //     ROS_DEBUG("Error transforming from frame '%s' to frame '%s'",
  //               msg_->header.frame_id.c_str(), qPrintable(fixed_frame_));
  //   }
  //   Ogre::Matrix4 transform(orientation);
  //   transform.setTrans(position);

  //   // Visualization of the end-effector trajectory
  //   // Getting the end-effector trajectory style
  //   uint32_t num_points = msg_->trajectory.size();
  //   uint32_t num_base = msg_->trajectory[0].base.size();
  //   LineStyle contact_style =
  //   (LineStyle)contact_style_property_->getOptionInt();

  //   // Getting the end-effector trajectory color
  //   Ogre::ColourValue contact_color =
  //   contact_color_property_->getOgreColor(); contact_color.a =
  //   contact_alpha_property_->getFloat();

  //   // Getting the number of contact trajectories
  //   uint32_t num_traj = 0;
  //   std::map<std::string, uint32_t> contact_traj_id;
  //   for (uint32_t i = 0; i < num_points; ++i) {
  //     uint32_t num_contacts = msg_->trajectory[i].contacts.size();
  //     for (uint32_t k = 0; k < num_contacts; k++) {
  //       dwl_msgs::ContactState contact = msg_->trajectory[i].contacts[k];

  //       if (contact_traj_id.find(contact.name) ==
  //           contact_traj_id.end()) { // a new swing trajectory
  //         contact_traj_id[contact.name] = num_traj;

  //         // Incrementing the counter (id) of swing trajectories
  //         num_traj++;
  //       }
  //     }
  //   }

  //   // Visualizing the different end-effector trajectories
  //   contact_traj_id.clear();
  //   std::map<uint32_t, uint32_t> contact_vec_id;
  //   switch (contact_style) {
  //   case LINES: {
  //     contact_manual_object_.clear();
  //     contact_manual_object_.resize(num_traj);

  //     uint32_t traj_id = 0;
  //     for (uint32_t i = 0; i < num_points; ++i) {
  //       uint32_t num_contacts = msg_->trajectory[i].contacts.size();
  //       for (uint32_t k = 0; k < num_contacts; k++) {
  //         dwl_msgs::ContactState contact = msg_->trajectory[i].contacts[k];

  //         if (contact_traj_id.find(contact.name) ==
  //             contact_traj_id.end()) { // a new swing trajectory
  //           contact_traj_id[contact.name] = traj_id;
  //           contact_vec_id[traj_id] = k;

  //           contact_manual_object_[traj_id].reset(
  //               scene_manager_->createManualObject());
  //           contact_manual_object_[traj_id]->setDynamic(true);
  //           scene_node_->attachObject(contact_manual_object_[traj_id].get());

  //           contact_manual_object_[traj_id]->estimateVertexCount(num_points);
  //           contact_manual_object_[traj_id]->begin(
  //               "BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP);

  //           // Incrementing the counter (id) of swing trajectories
  //           traj_id++;
  //         } else {
  //           uint32_t swing_idx = contact_traj_id.find(contact.name)->second;

  //           if (k != contact_vec_id.find(swing_idx)
  //                        ->second) { // change the vector index
  //             contact_vec_id[swing_idx] = k;
  //           }
  //         }
  //       }

  //       // Computing the actual base position
  //       Ogre::Vector3 base_pos(0., 0., 0.);
  //       Eigen::Vector3d base_rpy;
  //       for (uint32_t j = 0; j < num_base; j++) {
  //         dwl_msgs::BaseState base = msg_->trajectory[i].base[j];
  //         if (base.id == dwl::rbd::LX)
  //           base_pos.x = base.position;
  //         else if (base.id == dwl::rbd::LY)
  //           base_pos.y = base.position;
  //         else if (base.id == dwl::rbd::LZ)
  //           base_pos.z = base.position;
  //         else if (base.id == dwl::rbd::AX)
  //           base_rpy(0) = base.position;
  //         else if (base.id == dwl::rbd::AY)
  //           base_rpy(1) = base.position;
  //         else
  //           base_rpy(2) = base.position;
  //       }

  //       // Computing the base to world transform
  //       Eigen::Quaterniond quat = dwl::math::getQuaternion(base_rpy);
  //       Ogre::Quaternion ogre_quat(quat.w(), quat.x(), quat.y(), quat.z());
  //       Ogre::Matrix4 base_to_world_tf(ogre_quat);

  //       // Adding the contact points for the current swing trajectories
  //       for (std::map<std::string, uint32_t>::iterator traj_it =
  //                contact_traj_id.begin();
  //            traj_it != contact_traj_id.end(); traj_it++) {
  //         uint32_t traj_id = traj_it->second;
  //         uint32_t id = contact_vec_id.find(traj_id)->second;

  //         if (id < num_contacts) {
  //           dwl_msgs::ContactState contact =
  //           msg_->trajectory[i].contacts[id];

  //           Ogre::Vector3 xpos =
  //               base_pos + base_to_world_tf *
  //               Ogre::Vector3(contact.position.x,
  //                                                           contact.position.y,
  //                                                           contact.position.z);

  //           // sanity check orientation
  //           if (!(std::isfinite(xpos.x) && std::isfinite(xpos.y) &&
  //                 std::isfinite(xpos.z))) {
  //             std::cerr << "whole body contact trajectory is not finite, "
  //                          "resetting to zero!"
  //                       << std::endl;
  //             xpos.x = 0.0;
  //             xpos.y = 0.0;
  //             xpos.z = 0.0;
  //           }
  //           contact_manual_object_[traj_id]->position(xpos.x, xpos.y,
  //           xpos.z); contact_manual_object_[traj_id]->colour(contact_color);
  //         }
  //       }
  //     }
  //     // Ending the contact manual objects
  //     for (uint32_t i = 0; i < traj_id; i++)
  //       contact_manual_object_[i]->end();

  //     break;
  //   }

  //   case BILLBOARDS: {
  //     // Getting the end-effector line width
  //     float contact_line_width = contact_line_width_property_->getFloat();
  //     contact_billboard_line_.clear();
  //     contact_billboard_line_.resize(num_traj);

  //     uint32_t traj_id = 0;
  //     for (uint32_t i = 0; i < num_points; ++i) {
  //       uint32_t num_contacts = msg_->trajectory[i].contacts.size();
  //       for (uint32_t k = 0; k < num_contacts; k++) {
  //         dwl_msgs::ContactState contact = msg_->trajectory[i].contacts[k];

  //         if (contact_traj_id.find(contact.name) ==
  //             contact_traj_id.end()) { // a new swing trajectory
  //           contact_traj_id[contact.name] = traj_id;
  //           contact_vec_id[traj_id] = k;

  //           contact_billboard_line_[traj_id].reset(
  //               new rviz::BillboardLine(scene_manager_, scene_node_));
  //           contact_billboard_line_[traj_id]->setNumLines(1);
  //           contact_billboard_line_[traj_id]->setMaxPointsPerLine(num_points);
  //           contact_billboard_line_[traj_id]->setLineWidth(contact_line_width);

  //           // Incrementing the counter (id) of swing trajectories
  //           traj_id++;
  //         } else {
  //           uint32_t swing_idx = contact_traj_id.find(contact.name)->second;

  //           if (k != contact_vec_id.find(swing_idx)
  //                        ->second) { // change the vector index
  //             contact_vec_id[swing_idx] = k;
  //           }
  //         }
  //       }

  //       // Computing the actual base position
  //       Ogre::Vector3 base_pos(0., 0., 0.);
  //       Eigen::Vector3d base_rpy;
  //       for (uint32_t j = 0; j < num_base; j++) {
  //         dwl_msgs::BaseState base = msg_->trajectory[i].base[j];
  //         if (base.id == dwl::rbd::LX)
  //           base_pos.x = base.position;
  //         else if (base.id == dwl::rbd::LY)
  //           base_pos.y = base.position;
  //         else if (base.id == dwl::rbd::LZ)
  //           base_pos.z = base.position;
  //         else if (base.id == dwl::rbd::AX)
  //           base_rpy(0) = base.position;
  //         else if (base.id == dwl::rbd::AY)
  //           base_rpy(1) = base.position;
  //         else
  //           base_rpy(2) = base.position;
  //       }

  //       // Computing the base to world transform
  //       Eigen::Quaterniond quat = dwl::math::getQuaternion(base_rpy);
  //       Ogre::Quaternion ogre_quat(quat.w(), quat.x(), quat.y(), quat.z());
  //       Ogre::Matrix4 base_to_world_tf(ogre_quat);

  //       // Adding the contact points for the current swing trajectories
  //       for (std::map<std::string, uint32_t>::iterator traj_it =
  //                contact_traj_id.begin();
  //            traj_it != contact_traj_id.end(); traj_it++) {
  //         uint32_t traj_id = traj_it->second;
  //         uint32_t id = contact_vec_id.find(traj_id)->second;

  //         if (id < num_contacts) {
  //           dwl_msgs::ContactState contact =
  //           msg_->trajectory[i].contacts[id];

  //           Ogre::Vector3 xpos =
  //               base_pos + base_to_world_tf *
  //               Ogre::Vector3(contact.position.x,
  //                                                           contact.position.y,
  //                                                           contact.position.z);

  //           // sanity check orientation
  //           if (!(std::isfinite(xpos.x) && std::isfinite(xpos.y) &&
  //                 std::isfinite(xpos.z))) {
  //             std::cerr << "whole body contact trajectory is not finite, "
  //                          "resetting to zero!"
  //                       << std::endl;
  //             xpos.x = 0.0;
  //             xpos.y = 0.0;
  //             xpos.z = 0.0;
  //           }
  //           contact_billboard_line_[traj_id]->addPoint(xpos, contact_color);
  //         }
  //       }
  //     }
  //     break;
  //   }

  //   case POINTS: {
  //     // Getting the end-effector line width
  //     float contact_line_width = contact_line_width_property_->getFloat();
  //     contact_points_.clear();
  //     contact_points_.resize(num_points);

  //     uint32_t traj_id = 0;
  //     for (uint32_t i = 0; i < num_points; ++i) {
  //       uint32_t num_contacts = msg_->trajectory[i].contacts.size();
  //       for (uint32_t k = 0; k < num_contacts; k++) {
  //         dwl_msgs::ContactState contact = msg_->trajectory[i].contacts[k];
  //         if (contact_traj_id.find(contact.name) ==
  //             contact_traj_id.end()) { // a new swing trajectory
  //           contact_traj_id[contact.name] = traj_id;
  //           contact_vec_id[traj_id] = k;

  //           // Incrementing the counter (id) of swing trajectories
  //           traj_id++;
  //         } else {
  //           uint32_t swing_idx = contact_traj_id.find(contact.name)->second;

  //           if (k != contact_vec_id.find(swing_idx)
  //                        ->second) { // change the vector index
  //             contact_vec_id[swing_idx] = k;
  //           }
  //         }
  //       }

  //       // Updating the size
  //       contact_points_[i].clear();
  //       contact_points_[i].resize(num_contacts);

  //       // Computing the actual base position
  //       Ogre::Vector3 base_pos(0., 0., 0.);
  //       Eigen::Vector3d base_rpy;
  //       for (uint32_t j = 0; j < num_base; j++) {
  //         dwl_msgs::BaseState base = msg_->trajectory[i].base[j];
  //         if (base.id == dwl::rbd::LX)
  //           base_pos.x = base.position;
  //         else if (base.id == dwl::rbd::LY)
  //           base_pos.y = base.position;
  //         else if (base.id == dwl::rbd::LZ)
  //           base_pos.z = base.position;
  //         else if (base.id == dwl::rbd::AX)
  //           base_rpy(0) = base.position;
  //         else if (base.id == dwl::rbd::AY)
  //           base_rpy(1) = base.position;
  //         else
  //           base_rpy(2) = base.position;
  //       }

  //       // Computing the base to world transform
  //       Eigen::Quaterniond quat = dwl::math::getQuaternion(base_rpy);
  //       Ogre::Quaternion ogre_quat(quat.w(), quat.x(), quat.y(), quat.z());
  //       Ogre::Matrix4 base_to_world_tf(ogre_quat);

  //       // Adding the contact points for the current swing trajectories
  //       uint32_t contact_idx = 0;
  //       for (std::map<std::string, uint32_t>::iterator traj_it =
  //                contact_traj_id.begin();
  //            traj_it != contact_traj_id.end(); traj_it++) {
  //         uint32_t traj_id = traj_it->second;
  //         uint32_t id = contact_vec_id.find(traj_id)->second;

  //         if (id < num_contacts) {
  //           dwl_msgs::ContactState contact =
  //           msg_->trajectory[i].contacts[id];

  //           Ogre::Vector3 xpos =
  //               base_pos + base_to_world_tf *
  //               Ogre::Vector3(contact.position.x,
  //                                                           contact.position.y,
  //                                                           contact.position.z);

  //           // sanity check orientation
  //           if (!(std::isfinite(xpos.x) && std::isfinite(xpos.y) &&
  //                 std::isfinite(xpos.z))) {
  //             std::cerr << "whole body contact trajectory is not finite, "
  //                          "resetting to zero!"
  //                       << std::endl;
  //             xpos.x = 0.0;
  //             xpos.y = 0.0;
  //             xpos.z = 0.0;
  //           }
  //           contact_points_[i][contact_idx].reset(
  //               new PointVisual(context_->getSceneManager(), scene_node_));
  //           contact_points_[i][contact_idx]->setColor(
  //               contact_color.r, contact_color.g, contact_color.b,
  //               contact_color.a);
  //           contact_points_[i][contact_idx]->setRadius(contact_line_width);
  //           contact_points_[i][contact_idx]->setPoint(xpos);
  //           contact_points_[i][contact_idx]->setFramePosition(position);
  //           contact_points_[i][contact_idx]->setFrameOrientation(orientation);

  //           ++contact_idx;
  //         }
  //       }
  //     }
  //     break;
  //   }
}

void WholeBodyTrajectoryDisplay::destroyObjects() {
  com_manual_object_.reset();
  com_billboard_line_.reset();
  com_points_.clear();
  com_axes_.clear();
  contact_manual_object_.clear();
  contact_billboard_line_.clear();
  for (uint32_t i = 0; i < contact_points_.size(); i++)
    contact_points_[i].clear();
  contact_points_.clear();
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

} // namespace state_rviz_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(state_rviz_plugin::WholeBodyTrajectoryDisplay,
                       rviz::Display)
