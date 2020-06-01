#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreVector3.h>

#include <rviz/ogre_helpers/arrow.h>
#include <state_rviz_plugin/ArrowVisual.h>

namespace state_rviz_plugin {

ArrowVisual::ArrowVisual(Ogre::SceneManager *scene_manager,
                         Ogre::SceneNode *parent_node) {
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

ArrowVisual::~ArrowVisual() {
  // Delete the arrow to make it disappear.
  delete arrow_;

  // Destroy the frame node since we don't need it anymore.
  scene_manager_->destroySceneNode(frame_node_);
}

void ArrowVisual::setArrow(const Ogre::Vector3 &position,
                           const Ogre::Quaternion &orientation) {
  arrow_->setPosition(position);
  arrow_->setOrientation(orientation);
}

void ArrowVisual::setFramePosition(const Ogre::Vector3 &position) {
  frame_node_->setPosition(position);
}

void ArrowVisual::setFrameOrientation(const Ogre::Quaternion &orientation) {
  frame_node_->setOrientation(orientation);
}

void ArrowVisual::setColor(float r, float g, float b, float a) {
  arrow_->setColor(r, g, b, a);
}

void ArrowVisual::setProperties(float shaft_length, float shaft_diameter,
                                float head_length, float head_diameter) {
  arrow_->set(shaft_length, shaft_diameter, head_length, head_diameter);
}

} // namespace state_rviz_plugin
