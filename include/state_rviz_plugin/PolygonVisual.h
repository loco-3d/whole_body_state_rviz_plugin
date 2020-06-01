#ifndef STATE_RVIZ_PLUGIN_POLYGON_VISUAL_H
#define STATE_RVIZ_PLUGIN_POLYGON_VISUAL_H

#include "state_rviz_plugin/LineVisual.h"
#include <rviz/ogre_helpers/mesh_shape.h>
#include <rviz/properties/quaternion_property.h>

namespace Ogre {
class Vector3;
class Quaternion;
} // namespace Ogre

namespace state_rviz_plugin {

struct Triangle {
  Triangle(unsigned int _v1, unsigned int _v2, unsigned int _v3)
      : v1(_v1), v2(_v2), v3(_v3) {}

  unsigned int v1;
  unsigned int v2;
  unsigned int v3;
};

/**
 * @class PolygonVisual
 * @brief Visualizes the whole set of polygons given a vector of 3d points
 * Each instance of PolygonVisual represents the visualization of a set of
 * polygons data. Currently it just shows a set of polygons
 */
class PolygonVisual {
public:
  /**
   * @brief Constructor that creates the visual stuff and puts it into the scene
   * @param scene_manager  Manager the organization and rendering of the scene
   * @param parent_node    Represent the arrow as node in the scene
   */
  PolygonVisual(Ogre::SceneManager *scene_manager,
                Ogre::SceneNode *parent_node);

  /** @brief Destructor that removes the visual stuff from the scene */
  ~PolygonVisual();

  /**
   * @brief Configure the visual to show the polygons
   * @param polygon  Vertices of the polygon
   */
  void setVertices(std::vector<Ogre::Vector3> &polygon);

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
   * @brief Set the line color and alpha, which are user-editable
   * @param r  Red value
   * @param g  Green value
   * @param b  Blue value
   * @param a  Alpha value
   */
  void setLineColor(float r, float g, float b, float a);

  /**
   * @brief Set the mesh color and alpha, which are user-editable
   * @param r  Red value
   * @param g  Green value
   * @param b  Blue value
   * @param a  Alpha value
   */
  void setMeshColor(float r, float g, float b, float a);

  /**
   * @brief Set the radius of the line
   * @param scale  Line radius
   */
  void setLineRadius(float scale);

private:
  /** @brief The object implementing the polygon mesh */
  boost::shared_ptr<rviz::MeshShape> mesh_;

  /** @brief The object implementing the lines */
  std::vector<boost::shared_ptr<state_rviz_plugin::LineVisual>> line_;

  /** @brief A SceneNode whose pose is set to match the coordinate frame */
  Ogre::SceneNode *frame_node_;

  /** @brief The SceneManager, kept here only so the destructor can ask
   * it to destroy the ``frame_node_``.
   */
  Ogre::SceneManager *scene_manager_;
};

} // namespace state_rviz_plugin

#endif // STATE_RVIZ_PLUGIN_POLYGON_VISUAL_H
