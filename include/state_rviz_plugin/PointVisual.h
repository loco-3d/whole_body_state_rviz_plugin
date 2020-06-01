///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2018-2020, University of Edinburgh
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef STATE_RVIZ_PLUGIN_POINT_VISUAL_H
#define STATE_RVIZ_PLUGIN_POINT_VISUAL_H

namespace Ogre {
class Vector3;
class Quaternion;
} // namespace Ogre

namespace rviz {
class Shape;
}

namespace state_rviz_plugin {

/**
 * @class PointVisual
 * @brief Visualizes 3d point
 * Each instance of PointVisual represents the visualization of a single
 * Ogre::Vector3 data. Currently it just shows a sphere in the point position
 */
class PointVisual {
public:
  /**
   * @brief Constructor that creates the visual stuff and puts it into the scene
   * @param scene_manager  Manager the organization and rendering of the scene
   * @param parent_node    Represent the point as node in the scene
   */
  PointVisual(Ogre::SceneManager *scene_manager, Ogre::SceneNode *parent_node);

  /** @brief Destructor that removes the visual stuff from the scene */
  ~PointVisual();

  /**
   * @brief Configure the visual to show the point
   * @param point  Point position
   */
  void setPoint(const Ogre::Vector3 &point);

  /**
   * @brief Set the position of the coordinate frame
   * @param position  Frame position
   */
  void setFramePosition(const Ogre::Vector3 &position);

  /**
   * @brief Set the orientation of the coordinate frame
   * @param orientation  Frame orientation
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
   * @brief Set the radius of the point
   * @param r  Radius value
   */
  void setRadius(float r);

private:
  /** @brief The object implementing the point circle */
  rviz::Shape *point_;

  /** @brief A SceneNode whose pose is set to match the coordinate frame */
  Ogre::SceneNode *frame_node_;

  /** @brief The SceneManager, kept here only so the destructor can ask it to
   * destroy the ``frame_node_``.
   */
  Ogre::SceneManager *scene_manager_;

  /** @brief Radius value */
  float radius_;
};

} // namespace state_rviz_plugin

#endif // STATE_RVIZ_PLUGIN_POINT_VISUAL_H
