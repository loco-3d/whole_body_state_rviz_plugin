///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2020, University of Edinburgh
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "state_rviz_plugin/PinocchioLinkUpdater.h"
#include <rviz/frame_manager.h>

#include <tf/tf.h>

#include <OgreQuaternion.h>
#include <OgreVector3.h>
#include <pinocchio/algorithm/frames.hpp>

namespace state_rviz_plugin {

PinocchioLinkUpdater::PinocchioLinkUpdater(
    pinocchio::Model &model, pinocchio::Data &data,
    const Eigen::Ref<const Eigen::VectorXd> &q, const StatusCallback &status_cb,
    const std::string &tf_prefix)
    : model_(model), data_(data), status_callback_(status_cb),
      tf_prefix_(tf_prefix) {
  pinocchio::framesForwardKinematics(model_, data_, q);
}

bool PinocchioLinkUpdater::getLinkTransforms(
    const std::string &link_name, Ogre::Vector3 &visual_position,
    Ogre::Quaternion &visual_orientation, Ogre::Vector3 &collision_position,
    Ogre::Quaternion &collision_orientation) const {
  if (model_.existFrame(link_name)) {
    pinocchio::FrameIndex frameId = model_.getFrameId(link_name);
    const Eigen::Vector3d &translation = data_.oMf[frameId].translation();
    Eigen::Quaterniond quaternion(data_.oMf[frameId].rotation());
    Ogre::Vector3 position(translation[0], translation[1], translation[2]);
    Ogre::Quaternion orientation(quaternion.w(), quaternion.x(), quaternion.y(),
                                 quaternion.z());

    // Collision/visual transforms are the same in this case
    visual_position = position;
    visual_orientation = orientation;
    collision_position = position;
    collision_orientation = orientation;
    setLinkStatus(rviz::StatusProperty::Ok, link_name, "Frame OK");
  } else {
    std::stringstream ss;
    ss << "Do not exist frame [" << link_name << "]";
    setLinkStatus(rviz::StatusProperty::Error, link_name, ss.str());
    return false;
  }
  return true;
}

void PinocchioLinkUpdater::setLinkStatus(rviz::StatusLevel level,
                                         const std::string &link_name,
                                         const std::string &text) const {
  if (status_callback_) {
    status_callback_(level, link_name, text);
  }
}

} // namespace state_rviz_plugin