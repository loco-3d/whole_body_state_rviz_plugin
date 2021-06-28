///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2020, University of Edinburgh, Istituto Italiano di Tecnologia
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreVector3.h>

#include "whole_body_state_rviz_plugin/LineVisual.h"
#include <rviz/ogre_helpers/arrow.h>

namespace whole_body_state_rviz_plugin {

LineVisual::LineVisual(Ogre::SceneManager *scene_manager, Ogre::SceneNode *parent_node) : distance_(0.) {
  scene_manager_ = scene_manager;

  // Ogre::SceneNode s form a tree, with each node storing the transform
  // (position and orientation) of itself relative to its parent. Ogre does
  // the math of combining those transforms when it is time to render.
  // Here we create a node to store the pose of the Point's header frame
  // relative to the RViz fixed frame.
  frame_node_ = parent_node->createChildSceneNode();

  // We create the arrow object within the frame node so that we can set its
  // position and direction relative to its header frame.
  arrow_ = new rviz::Arrow(scene_manager_, frame_node_);
}

LineVisual::~LineVisual() {
  // Delete the arrow to make it disappear.
  delete arrow_;
  // Destroy the frame node since we don't need it anymore.
  scene_manager_->destroySceneNode(frame_node_);
}

void LineVisual::setArrow(const Ogre::Vector3 &initial_point, const Ogre::Vector3 &final_point) {
  arrow_->setPosition(initial_point);
  Eigen::Vector3d ref_dir = -Eigen::Vector3d::UnitZ();
  Eigen::Vector3d arrow_dir(final_point.x - initial_point.x, final_point.y - initial_point.y,
                            final_point.z - initial_point.z);

  Eigen::Quaterniond arrow_q;
  arrow_q.setFromTwoVectors(ref_dir, arrow_dir);
  Ogre::Quaternion orientation(arrow_q.w(), arrow_q.x(), arrow_q.y(), arrow_q.z());
  arrow_->setOrientation(orientation);
  distance_ = arrow_dir.norm();
}

void LineVisual::setFramePosition(const Ogre::Vector3 &position) { frame_node_->setPosition(position); }

void LineVisual::setFrameOrientation(const Ogre::Quaternion &orientation) { frame_node_->setOrientation(orientation); }

void LineVisual::setColor(float r, float g, float b, float a) { arrow_->setColor(r, g, b, a); }

void LineVisual::setProperties(float shaft_diameter, float head_length, float head_diameter) {
  arrow_->set(distance_, shaft_diameter, head_length, head_diameter);
}

}  // namespace whole_body_state_rviz_plugin
