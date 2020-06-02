///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2018-2020, University of Edinburgh
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include <Eigen/Dense>

#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <rviz/frame_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/validate_floats.h>
#include <rviz/visualization_manager.h>

#include "state_rviz_plugin/WholeBodyStateDisplay.h"

using namespace rviz;

namespace state_rviz_plugin {

WholeBodyStateDisplay::WholeBodyStateDisplay()
    : is_info_(false), initialized_model_(false), force_threshold_(0.),
      weight_(0.), gravity_(9.81), com_real_(true) {
  // Robot properties
  robot_model_property_ =
      new StringProperty("Robot Description", "robot_description",
                         "Name of the parameter to search for to load"
                         " the robot description.",
                         this, SLOT(updateRobotModel()));

  // Category Groups
  com_category_ = new rviz::Property("Center Of Mass", QVariant(), "", this);
  cop_category_ =
      new rviz::Property("Center Of Pressure", QVariant(), "", this);
  icp_category_ =
      new rviz::Property("Instantaneous Capture Point", QVariant(), "", this);
  cmp_category_ =
      new rviz::Property("Centroidal Momentum Pivot", QVariant(), "", this);
  grf_category_ = new rviz::Property("Contact Forces", QVariant(), "", this);
  support_category_ =
      new rviz::Property("Support Region", QVariant(), "", this);
  friction_category_ =
      new rviz::Property("Friction Cone", QVariant(), "", this);

  // CoM position and velocity properties
  com_style_property_ = new EnumProperty(
      "CoM Style", "Real", "The rendering operation to use to draw the CoM.",
      com_category_, SLOT(updateCoMStyle()), this);
  com_style_property_->addOption("Real", REAL);
  com_style_property_->addOption("Projected", PROJECTED);
  com_color_property_ = new rviz::ColorProperty(
      "Color", QColor(255, 85, 0), "Color of a point", com_category_,
      SLOT(updateCoMColorAndAlpha()), this);
  com_alpha_property_ = new rviz::FloatProperty(
      "Alpha", 1.0, "0 is fully transparent, 1.0 is fully opaque.",
      com_category_, SLOT(updateCoMColorAndAlpha()), this);
  com_alpha_property_->setMin(0);
  com_alpha_property_->setMax(1);
  com_radius_property_ = new rviz::FloatProperty(
      "Radius", 0.04, "Radius of a point", com_category_,
      SLOT(updateCoMColorAndAlpha()), this);
  com_shaft_length_property_ = new FloatProperty(
      "Shaft Length", 0.4, "Length of the arrow's shaft, in meters.",
      com_category_, SLOT(updateCoMArrowGeometry()), this);
  com_shaft_radius_property_ = new FloatProperty(
      "Shaft Radius", 0.02, "Radius of the arrow's shaft, in meters.",
      com_category_, SLOT(updateCoMArrowGeometry()), this);
  com_head_length_property_ = new FloatProperty(
      "Head Length", 0.08, "Length of the arrow's head, in meters.",
      com_category_, SLOT(updateCoMArrowGeometry()), this);
  com_head_radius_property_ = new FloatProperty(
      "Head Radius", 0.04, "Radius of the arrow's head, in meters.",
      com_category_, SLOT(updateCoMArrowGeometry()), this);

  // CoP properties
  cop_color_property_ = new rviz::ColorProperty(
      "Color", QColor(204, 41, 204), "Color of a point", cop_category_,
      SLOT(updateCoPColorAndAlpha()), this);
  cop_alpha_property_ = new rviz::FloatProperty(
      "Alpha", 1.0, "0 is fully transparent, 1.0 is fully opaque.",
      cop_category_, SLOT(updateCoPColorAndAlpha()), this);
  cop_alpha_property_->setMin(0);
  cop_alpha_property_->setMax(1);
  cop_radius_property_ = new rviz::FloatProperty(
      "Radius", 0.04, "Radius of a point", cop_category_,
      SLOT(updateCoPColorAndAlpha()), this);

  // Instantaneous Capture Point properties
  icp_color_property_ = new rviz::ColorProperty(
      "Color", QColor(10, 41, 10), "Color of a point", icp_category_,
      SLOT(updateICPColorAndAlpha()), this);
  icp_alpha_property_ = new rviz::FloatProperty(
      "Alpha", 1.0, "0 is fully transparent, 1.0 is fully opaque.",
      icp_category_, SLOT(updateICPColorAndAlpha()), this);
  icp_alpha_property_->setMin(0);
  icp_alpha_property_->setMax(1);
  icp_radius_property_ = new rviz::FloatProperty(
      "Radius", 0.04, "Radius of a point", icp_category_,
      SLOT(updateICPColorAndAlpha()), this);

  // CMP properties
  cmp_color_property_ = new rviz::ColorProperty(
      "Color", QColor(200, 41, 10), "Color of a point", cmp_category_,
      SLOT(updateCMPColorAndAlpha()), this);
  cmp_alpha_property_ = new rviz::FloatProperty(
      "Alpha", 1.0, "0 is fully transparent, 1.0 is fully opaque.",
      cmp_category_, SLOT(updateCMPColorAndAlpha()), this);
  cmp_alpha_property_->setMin(0);
  cmp_alpha_property_->setMax(1);
  cmp_radius_property_ = new rviz::FloatProperty(
      "Radius", 0.04, "Radius of a point", cmp_category_,
      SLOT(updateCMPColorAndAlpha()), this);

  // GRF properties
  grf_color_property_ =
      new ColorProperty("Color", QColor(85, 0, 255), "Color to draw the arrow.",
                        grf_category_, SLOT(updateGRFColorAndAlpha()), this);
  grf_alpha_property_ = new FloatProperty(
      "Alpha", 1.0, "Amount of transparency to apply to the arrow.",
      grf_category_, SLOT(updateGRFColorAndAlpha()), this);
  grf_alpha_property_->setMin(0);
  grf_alpha_property_->setMax(1);
  grf_shaft_length_property_ = new FloatProperty(
      "Shaft Length", 0.8, "Length of the arrow's shaft, in meters.",
      grf_category_, SLOT(updateGRFArrowGeometry()), this);
  grf_shaft_radius_property_ = new FloatProperty(
      "Shaft Radius", 0.02, "Radius of the arrow's shaft, in meters.",
      grf_category_, SLOT(updateGRFArrowGeometry()), this);
  grf_head_length_property_ = new FloatProperty(
      "Head Length", 0.08, "Length of the arrow's head, in meters.",
      grf_category_, SLOT(updateGRFArrowGeometry()), this);
  grf_head_radius_property_ = new FloatProperty(
      "Head Radius", 0.04, "Radius of the arrow's head, in meters.",
      grf_category_, SLOT(updateGRFArrowGeometry()), this);

  // Support region properties
  support_line_color_property_ = new ColorProperty(
      "Line Color", QColor(85, 0, 255), "Color to draw the line.",
      support_category_, SLOT(updateSupportLineColorAndAlpha()), this);
  support_line_alpha_property_ = new FloatProperty(
      "Line Alpha", 1.0, "Amount of transparency to apply to the line.",
      support_category_, SLOT(updateSupportLineColorAndAlpha()), this);
  support_line_alpha_property_->setMin(0);
  support_line_alpha_property_->setMax(1);
  support_line_radius_property_ = new FloatProperty(
      "Line Radius", 0.005, "Radius of the line in m.", support_category_,
      SLOT(updateSupportLineColorAndAlpha()), this);
  support_mesh_color_property_ = new ColorProperty(
      "Mesh Color", QColor(85, 0, 255), "Color to draw the mesh.",
      support_category_, SLOT(updateSupportMeshColorAndAlpha()), this);
  support_mesh_alpha_property_ = new FloatProperty(
      "Mesh Alpha", 0.2, "Amount of transparency to apply to the mesh.",
      support_category_, SLOT(updateSupportMeshColorAndAlpha()), this);
  support_mesh_alpha_property_->setMin(0);
  support_mesh_alpha_property_->setMax(1);
  support_force_threshold_property_ = new FloatProperty(
      "Force Threshold", 1.0, "Threshold for defining active contacts.",
      support_category_, SLOT(updateSupportLineColorAndAlpha()), this);

  // Friction cone properties
  friction_cone_color_property_ = new ColorProperty(
      "Color", QColor(255, 0, 127), "Color to draw the friction cone.",
      friction_category_, SLOT(updateFrictionConeColorAndAlpha()), this);
  friction_cone_alpha_property_ = new FloatProperty(
      "Alpha", 0.5, "Amount of transparency to apply to the friction cone.",
      friction_category_, SLOT(updateFrictionConeColorAndAlpha()), this);
  friction_cone_alpha_property_->setMin(0);
  friction_cone_alpha_property_->setMax(1);
  friction_cone_length_property_ = new FloatProperty(
      "Length", 0.2, "Length of the friction cone in m.", friction_category_,
      SLOT(updateFrictionConeGeometry()), this);
}

WholeBodyStateDisplay::~WholeBodyStateDisplay() {}

void WholeBodyStateDisplay::clear() {
  clearStatuses();
  robot_model_.clear();
  model_ = pinocchio::Model();
  initialized_model_ = false;
}

void WholeBodyStateDisplay::onInitialize() {
  MFDClass::onInitialize();
  updateGRFColorAndAlpha();
}

void WholeBodyStateDisplay::onEnable() {
  MFDClass::onEnable();
  load();
}

void WholeBodyStateDisplay::onDisable() {
  MFDClass::onDisable();
  clear();
}

void WholeBodyStateDisplay::fixedFrameChanged() {
  if (is_info_) {
    processWholeBodyState();
  }
}

void WholeBodyStateDisplay::reset() {
  MFDClass::reset();
  grf_visual_.clear();
  cones_visual_.clear();
}

void WholeBodyStateDisplay::load() {
  std::string content;
  if (!update_nh_.getParam(robot_model_property_->getStdString(), content)) {
    std::string loc;
    if (update_nh_.searchParam(robot_model_property_->getStdString(), loc)) {
      update_nh_.getParam(loc, content);
    } else {
      clear();
      setStatus(StatusProperty::Error, "URDF",
                "Parameter [" + robot_model_property_->getString() +
                    "] does not exist, and was not found by searchParam()");
      return;
    }
  }

  if (content.empty()) {
    clear();
    setStatus(StatusProperty::Error, "URDF", "URDF is empty");
    return;
  }

  if (content == robot_model_) {
    return;
  }

  robot_model_ = content;

  // Initializing the dynamics from the URDF model
  pinocchio::urdf::buildModelFromXML(robot_model_,
                                     pinocchio::JointModelFreeFlyer(), model_);
  gravity_ = model_.gravity.linear().norm();
  weight_ = pinocchio::computeTotalMass(model_) * gravity_;
  initialized_model_ = true;
  setStatus(StatusProperty::Ok, "URDF", "URDF parsed OK");
}

void WholeBodyStateDisplay::updateRobotModel() {
  if (isEnabled()) {
    load();
    context_->queueRender();
  }
}

void WholeBodyStateDisplay::updateCoMStyle() {
  CoMStyle style = (CoMStyle)com_style_property_->getOptionInt();

  switch (style) {
  case REAL:
  default:
    com_real_ = true;
    break;

  case PROJECTED:
    com_real_ = false;
    break;
  }
}

void WholeBodyStateDisplay::updateCoMColorAndAlpha() {
  const float &radius = com_radius_property_->getFloat();
  Ogre::ColourValue color = com_color_property_->getOgreColor();
  color.a = com_alpha_property_->getFloat();
  if (com_visual_) {
    com_visual_->setColor(color.r, color.g, color.b, color.a);
    com_visual_->setRadius(radius);
  }
  if (comd_visual_) {
    comd_visual_->setColor(color.r, color.g, color.b, color.a);
  }
  context_->queueRender();
}

void WholeBodyStateDisplay::updateCoMArrowGeometry() {
  const float &shaft_length = com_shaft_length_property_->getFloat();
  const float &shaft_radius = com_shaft_radius_property_->getFloat();
  const float &head_length = com_head_length_property_->getFloat();
  const float &head_radius = com_head_radius_property_->getFloat();
  if (comd_visual_) {
    comd_visual_->setProperties(shaft_length, shaft_radius, head_length,
                                head_radius);
  }
  context_->queueRender();
}

void WholeBodyStateDisplay::updateCoPColorAndAlpha() {
  const float &radius = cop_radius_property_->getFloat();
  Ogre::ColourValue color = cop_color_property_->getOgreColor();
  color.a = cop_alpha_property_->getFloat();
  if (cop_visual_) {
    cop_visual_->setColor(color.r, color.g, color.b, color.a);
    cop_visual_->setRadius(radius);
  }
  context_->queueRender();
}

void WholeBodyStateDisplay::updateICPColorAndAlpha() {
  float radius = icp_radius_property_->getFloat();
  Ogre::ColourValue color = icp_color_property_->getOgreColor();
  color.a = icp_alpha_property_->getFloat();
  if (icp_visual_) {
    icp_visual_->setColor(color.r, color.g, color.b, color.a);
    icp_visual_->setRadius(radius);
  }
  context_->queueRender();
}

void WholeBodyStateDisplay::updateCMPColorAndAlpha() {
  const float &radius = cmp_radius_property_->getFloat();
  Ogre::ColourValue color = cmp_color_property_->getOgreColor();
  color.a = cmp_alpha_property_->getFloat();
  if (cmp_visual_) {
    cmp_visual_->setColor(color.r, color.g, color.b, color.a);
    cmp_visual_->setRadius(radius);
  }
  context_->queueRender();
}

void WholeBodyStateDisplay::updateGRFColorAndAlpha() {
  Ogre::ColourValue color = grf_color_property_->getOgreColor();
  color.a = grf_alpha_property_->getFloat();
  for (size_t i = 0; i < grf_visual_.size(); ++i) {
    grf_visual_[i]->setColor(color.r, color.g, color.b, color.a);
  }
  context_->queueRender();
}

void WholeBodyStateDisplay::updateGRFArrowGeometry() {
  const float &shaft_length = grf_shaft_length_property_->getFloat();
  const float &shaft_radius = grf_shaft_radius_property_->getFloat();
  const float &head_length = grf_head_length_property_->getFloat();
  const float &head_radius = grf_head_radius_property_->getFloat();
  for (size_t i = 0; i < grf_visual_.size(); ++i) {
    grf_visual_[i]->setProperties(shaft_length, shaft_radius, head_length,
                                  head_radius);
  }
  context_->queueRender();
}

void WholeBodyStateDisplay::updateSupportLineColorAndAlpha() {
  Ogre::ColourValue color = support_line_color_property_->getOgreColor();
  color.a = support_line_alpha_property_->getFloat();
  force_threshold_ = support_force_threshold_property_->getFloat();

  float radius = support_line_radius_property_->getFloat();
  if (support_visual_) {
    support_visual_->setLineColor(color.r, color.g, color.b, color.a);
    support_visual_->setLineRadius(radius);
  }
  context_->queueRender();
}

void WholeBodyStateDisplay::updateSupportMeshColorAndAlpha() {
  Ogre::ColourValue color = support_mesh_color_property_->getOgreColor();
  color.a = support_mesh_alpha_property_->getFloat();
  if (support_visual_) {
    support_visual_->setMeshColor(color.r, color.g, color.b, color.a);
  }
  context_->queueRender();
}

void WholeBodyStateDisplay::updateFrictionConeColorAndAlpha() {
  Ogre::ColourValue oc = friction_cone_color_property_->getOgreColor();
  float alpha = friction_cone_alpha_property_->getFloat();
  for (size_t i = 0; i < cones_visual_.size(); ++i) {
    cones_visual_[i]->setColor(oc.r, oc.g, oc.b, alpha);
  }
  context_->queueRender();
}

void WholeBodyStateDisplay::updateFrictionConeGeometry() {
  const float &cone_length = friction_cone_length_property_->getFloat();
  double cone_width = 2.0 * cone_length * tan(friction_mu_ / sqrt(2.));
  Ogre::Vector3 scale(cone_width, cone_length, cone_width);
  for (size_t i = 0; i < cones_visual_.size(); ++i) {
    cones_visual_[i]->setScale(scale);
    ;
  }
  context_->queueRender();
}

void WholeBodyStateDisplay::processMessage(
    const state_msgs::WholeBodyState::ConstPtr &msg) {
  msg_ = msg;
  is_info_ = true;
  processWholeBodyState();
}

void WholeBodyStateDisplay::processWholeBodyState() {
  // Checking if the urdf model was initialized
  if (!initialized_model_)
    return;

  // Here we call the rviz::FrameManager to get the transform from the
  // fixed frame to the frame in the header of this Point message.  If
  // it fails, we can't do anything else so we return.
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if (!context_->getFrameManager()->getTransform(
          msg_->header.frame_id, msg_->header.stamp, position, orientation)) {
    ROS_DEBUG("Error transforming from frame '%s' to frame '%s'",
              msg_->header.frame_id.c_str(), qPrintable(fixed_frame_));
    return;
  }

  // Resetting the point visualizers
  com_visual_.reset(new PointVisual(context_->getSceneManager(), scene_node_));
  comd_visual_.reset(new ArrowVisual(context_->getSceneManager(), scene_node_));
  support_visual_.reset(
      new PolygonVisual(context_->getSceneManager(), scene_node_));

  // Now set or update the contents of the chosen GRF visual
  std::vector<Ogre::Vector3> support;
  size_t num_contacts = msg_->contacts.size();
  size_t n_suppcontacts = 0;
  grf_visual_.clear();
  cones_visual_.clear();
  Eigen::Vector3d cop_pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d total_force = Eigen::Vector3d::Zero();
  for (size_t i = 0; i < num_contacts; ++i) {
    const state_msgs::ContactState &contact = msg_->contacts[i];
    std::string name = contact.name;

    // Getting the contact position
    Ogre::Vector3 contact_pos(contact.pose.position.x, contact.pose.position.y,
                              contact.pose.position.z);

    // Getting the force direction
    Eigen::Vector3d for_ref_dir = -Eigen::Vector3d::UnitZ();
    Eigen::Vector3d for_dir(contact.wrench.force.x, contact.wrench.force.y,
                            contact.wrench.force.z);

    // Updating the center of pressure
    if (contact.type == 0) {
      n_suppcontacts += 1;
      cop_pos +=
          contact.wrench.force.z * Eigen::Vector3d(contact.pose.position.x,
                                                   contact.pose.position.y,
                                                   contact.pose.position.z);
      total_force +=
          Eigen::Vector3d(contact.wrench.force.x, contact.wrench.force.y,
                          contact.wrench.force.z);
    }

    // Building the support polygone
    if (for_dir.norm() > force_threshold_ && std::isfinite(contact_pos.x) &&
        std::isfinite(contact_pos.y) && std::isfinite(contact_pos.z)) {
      Eigen::Quaterniond for_q;
      for_q.setFromTwoVectors(for_ref_dir, for_dir);
      Ogre::Quaternion contact_for_orientation(for_q.w(), for_q.x(), for_q.y(),
                                               for_q.z());

      // We are keeping a vector of visual pointers. This creates the next
      // one and stores it in the vector
      boost::shared_ptr<ArrowVisual> arrow;
      arrow.reset(new ArrowVisual(context_->getSceneManager(), scene_node_));
      arrow->setArrow(contact_pos, contact_for_orientation);
      arrow->setFramePosition(position);
      arrow->setFrameOrientation(orientation);

      // Setting the arrow color and properties
      Ogre::ColourValue color = grf_color_property_->getOgreColor();
      color.a = grf_alpha_property_->getFloat();
      arrow->setColor(color.r, color.g, color.b, color.a);
      const float &shaft_length =
          grf_shaft_length_property_->getFloat() * for_dir.norm() / weight_;
      const float &shaft_radius = grf_shaft_radius_property_->getFloat();
      const float &head_length = grf_head_length_property_->getFloat();
      const float &head_radius = grf_head_radius_property_->getFloat();
      arrow->setProperties(shaft_length, shaft_radius, head_length,
                           head_radius);

      // And send it to the end of the vector
      if (std::isfinite(shaft_length) && std::isfinite(shaft_radius) &&
          std::isfinite(head_length) && std::isfinite(head_radius)) {
        grf_visual_.push_back(arrow);
      }
      if (contact.type == 0) {
        support.push_back(contact_pos);
      }
    }

    // Building the friction cones
    Eigen::Vector3d cone_dir(contact.surface_normal.x, contact.surface_normal.y,
                             contact.surface_normal.z);
    friction_mu_ = contact.friction_coefficient;
    if (for_dir.norm() > force_threshold_ && cone_dir.norm() != 0 &&
        friction_mu_ != 0) {
      Eigen::Vector3d cone_ref_dir = -Eigen::Vector3d::UnitY();
      Eigen::Quaterniond cone_q;
      cone_q.setFromTwoVectors(cone_ref_dir, cone_dir);
      Ogre::Quaternion cone_orientation(cone_q.w(), cone_q.x(), cone_q.y(),
                                        cone_q.z());
      boost::shared_ptr<Shape> cone;
      cone.reset(new rviz::Shape(rviz::Shape::Cone, context_->getSceneManager(),
                                 scene_node_));
      cone->setPosition(contact_pos);
      cone->setOrientation(cone_orientation);

      double displayed_range = friction_cone_length_property_->getFloat();
      double cone_width = 2.0 * displayed_range * tan(friction_mu_ / sqrt(2.));
      Ogre::Vector3 scale(cone_width, displayed_range, cone_width);
      Ogre::ColourValue color = friction_cone_color_property_->getOgreColor();
      color.a = friction_cone_alpha_property_->getFloat();
      cone->setScale(scale);
      cone->setOffset(Ogre::Vector3(0, -0.5, 0.));
      cone->setColor(color.r, color.g, color.b, color.a);
      cones_visual_.push_back(cone);
    }
  }

  // Building the CoP visual
  if (n_suppcontacts != 0) {
    cop_pos /= total_force(2);
  }

  // Defining the center of mass as Ogre::Vector3
  Ogre::Vector3 com_point;
  if (com_real_ || n_suppcontacts != 0) {
    com_point.x = msg_->centroidal.com_position.x;
    com_point.y = msg_->centroidal.com_position.y;
    com_point.z = msg_->centroidal.com_position.z;
  } else {
    Eigen::Vector3d cop_z = Eigen::Vector3d::Zero();
    cop_z(2) = cop_pos(2);
    pinocchio::SE3::Quaternion q(msg_->centroidal.base_orientation.w,
                                 msg_->centroidal.base_orientation.x,
                                 msg_->centroidal.base_orientation.y,
                                 msg_->centroidal.base_orientation.z);
    Eigen::Vector3d rot_cop_z = q.matrix() * cop_z;
    com_point.x = msg_->centroidal.com_position.x + rot_cop_z(0);
    com_point.y = msg_->centroidal.com_position.y + rot_cop_z(1);
    com_point.z = cop_z(2);
  }

  // Defining the center of mass velocity orientation
  Eigen::Vector3d com_ref_dir = -Eigen::Vector3d::UnitZ();
  Eigen::Vector3d com_vel(msg_->centroidal.com_velocity.x,
                          msg_->centroidal.com_velocity.y,
                          msg_->centroidal.com_velocity.z);
  Eigen::Quaterniond com_q;
  com_q.setFromTwoVectors(com_ref_dir, com_vel);
  Ogre::Quaternion comd_for_orientation(com_q.w(), com_q.x(), com_q.y(),
                                        com_q.z());

  // Now set or update the contents of the chosen CoM visual
  updateCoMColorAndAlpha();
  if (std::isfinite(com_point.x) && std::isfinite(com_point.y) &&
      std::isfinite(com_point.z)) {
    com_visual_->setPoint(com_point);
    com_visual_->setFramePosition(position);
    com_visual_->setFrameOrientation(orientation);
    const double &com_vel_norm = com_vel.norm();
    const float &shaft_length =
        com_shaft_length_property_->getFloat() * com_vel_norm;
    const float &shaft_radius = com_shaft_radius_property_->getFloat();
    float head_length = 0., head_radius = 0.;
    if (com_vel_norm > 0.01) {
      head_length = com_head_length_property_->getFloat();
      head_radius = com_head_radius_property_->getFloat();
    }
    comd_visual_->setProperties(shaft_length, shaft_radius, head_length,
                                head_radius);
    comd_visual_->setArrow(com_point, comd_for_orientation);
    comd_visual_->setFramePosition(position);
    comd_visual_->setFrameOrientation(orientation);
  }

  // Now set or update the contents of the chosen CoP visual
  if (n_suppcontacts != 0) {
    cop_visual_.reset(
        new PointVisual(context_->getSceneManager(), scene_node_));
    icp_visual_.reset(
        new PointVisual(context_->getSceneManager(), scene_node_));
    cmp_visual_.reset(
        new PointVisual(context_->getSceneManager(), scene_node_));

    updateCoPColorAndAlpha();
    if (std::isfinite(cop_pos(0)) && std::isfinite(cop_pos(1)) &&
        std::isfinite(cop_pos(2))) {
      Ogre::Vector3 cop_point(cop_pos(0), cop_pos(1), cop_pos(2));
      cop_visual_->setPoint(cop_point);
      cop_visual_->setFramePosition(position);
      cop_visual_->setFrameOrientation(orientation);
    }

    // Computing the ICP
    double height = abs(msg_->centroidal.com_position.z - cop_pos(2));
    double omega = sqrt(gravity_ / height);
    Eigen::Vector3d com_pos = Eigen::Vector3d(msg_->centroidal.com_position.x,
                                              msg_->centroidal.com_position.y,
                                              msg_->centroidal.com_position.z);
    Eigen::Vector3d icp_pos = com_pos + com_vel / omega;
    icp_pos(2) = cop_pos(2);

    // Now set or update the contents of the chosen Inst CP visual
    updateICPColorAndAlpha();
    if (std::isfinite(icp_pos(0)) && std::isfinite(icp_pos(1)) &&
        std::isfinite(icp_pos(2))) {
      Ogre::Vector3 icp_point(icp_pos(0), icp_pos(1), icp_pos(2));
      icp_visual_->setPoint(icp_point);
      icp_visual_->setFramePosition(position);
      icp_visual_->setFrameOrientation(orientation);
    }

    // Computing the CMP
    Eigen::Vector3d cmp_pos;
    cmp_pos(0) = com_pos(0) - total_force(0) / total_force(2) * height;
    cmp_pos(1) = com_pos(1) - total_force(1) / total_force(2) * height;
    cmp_pos(2) = com_pos(2) - height;
    updateCMPColorAndAlpha();
    if (std::isfinite(cmp_pos(0)) && std::isfinite(cmp_pos(1)) &&
        std::isfinite(cmp_pos(2))) {
      Ogre::Vector3 cmp_point(cmp_pos(0), cmp_pos(1), cmp_pos(2));
      cmp_visual_->setPoint(cmp_point);
      cmp_visual_->setFramePosition(position);
      cmp_visual_->setFrameOrientation(orientation);
    }
  } else {
    if (cop_visual_) {
      Ogre::ColourValue color = cop_color_property_->getOgreColor();
      color.a = 0.;
      cop_visual_->setColor(color.r, color.g, color.b, color.a);
    }
    if (icp_visual_) {
      Ogre::ColourValue color = icp_color_property_->getOgreColor();
      color.a = 0.;
      icp_visual_->setColor(color.r, color.g, color.b, color.a);
    }
    if (cmp_visual_) {
      Ogre::ColourValue color = cmp_color_property_->getOgreColor();
      color.a = 0.;
      cmp_visual_->setColor(color.r, color.g, color.b, color.a);
    }
  }

  // Now set or update the contents of the chosen CoP visual
  support_visual_->setVertices(support);
  updateSupportLineColorAndAlpha();
  updateSupportMeshColorAndAlpha();
  support_visual_->setFramePosition(position);
  support_visual_->setFrameOrientation(orientation);
}

} // namespace state_rviz_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(state_rviz_plugin::WholeBodyStateDisplay, rviz::Display)
