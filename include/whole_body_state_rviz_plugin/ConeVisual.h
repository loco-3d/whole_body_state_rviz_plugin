///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2021, University of Edinburgh
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef WHOLE_BODY_STATE_RVIZ_PLUGIN_CONE_VISUAL_H
#define WHOLE_BODY_STATE_RVIZ_PLUGIN_CONE_VISUAL_H

#include <rviz/properties/quaternion_property.h>

namespace Ogre {
class Vector3;
class Quaternion;
}  // namespace Ogre

namespace rviz {
class Shape;
}

namespace whole_body_state_rviz_plugin {

/**
 * @class ConeVisual
 * @brief Visualizes 3d cone
 * Each instance of ConeVisual represents the visualization of a single cone
 * data. Currently it just shows a cone with a given direction, coefficient and magnitude.
 */
class ConeVisual {
 public:
  /**
   * @brief Constructor that creates the visual stuff and puts it into the scene
   * @param scene_manager  Manager the organization and rendering of the scene
   * @param parent_node    Represent the cone as node in the scene
   */
  ConeVisual(Ogre::SceneManager *scene_manager, Ogre::SceneNode *parent_node);

  /** @brief Destructor that removes the visual stuff from the scene */
  ~ConeVisual();

  /**
   * @brief Configure the visual to show the cone
   * @param position     Cone position
   * @param orientation  Cone orientation
   */
  void setCone(const Ogre::Vector3 &position, const Ogre::Quaternion &orientation);

  /**
   * @brief Set the position of the coordinate frame
   * @param position  Frame position
   */
  void setFramePosition(const Ogre::Vector3 &position);

  /**
   * @brief Set the orientation of the coordinate frame
   * @param orientation Frame orientation
   */
  void setFrameOrientation(const Ogre::Quaternion &orientation);

  /**
   * @brief Set the color and alpha of the visual, which are user-editable
   * @param r  Red value
   * @param g  Green value
   * @param b  Blue value
   * @param a  Alpha value
   */
  void setColor(float r, float g, float b, float a);

  /**
   * @brief Set the parameters for this cone
   * @param width    Cone width
   * @param length  Cone length
   */
  void setProperties(float width, float length);

 private:
  /** @brief The object implementing the cone */
  rviz::Shape *cone_;

  /** @brief A SceneNode whose pose is set to match the coordinate frame */
  Ogre::SceneNode *frame_node_;

  /** @brief The SceneManager, kept here only so the destructor can ask it to
   * destroy the ``frame_node_``.
   */
  Ogre::SceneManager *scene_manager_;
};

}  // namespace whole_body_state_rviz_plugin

#endif  // WHOLE_BODY_STATE_RVIZ_PLUGIN_CONE_VISUAL_H
