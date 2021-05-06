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

#include "whole_body_state_rviz_plugin/PolygonVisual.h"
#include <rviz/ogre_helpers/line.h>

namespace whole_body_state_rviz_plugin {

PolygonVisual::PolygonVisual(Ogre::SceneManager *scene_manager,
                             Ogre::SceneNode *parent_node) {
  scene_manager_ = scene_manager;

  // Ogre::SceneNode s form a tree, with each node storing the transform
  // (position and orientation) of itself relative to its parent. Ogre does
  // the math of combining those transforms when it is time to render. Here
  // we create a node to store the pose of the Point's header frame relative
  // to the RViz fixed frame.
  frame_node_ = parent_node->createChildSceneNode();

  // Initialization of the mesh
  mesh_.reset(new rviz::MeshShape(scene_manager, parent_node));
}

PolygonVisual::~PolygonVisual() {
  // Delete the line and mesh to make it disappear.
  line_.clear();
  mesh_->clear();
  // Destroy the frame node since we don't need it anymore.
  scene_manager_->destroySceneNode(frame_node_);
}

void PolygonVisual::setVertices(std::vector<Ogre::Vector3> &vertices) {
  // Getting the number of vertices
  unsigned int num_vertex = vertices.size();

  // Visualization of the lines
  line_.clear();
  unsigned int num_line = 0;
  for (unsigned int i = 1; i < num_vertex; i++)
    num_line += num_vertex - i;
  line_.resize(num_line);

  unsigned int counter = 0;
  unsigned int tree = num_vertex;
  while (tree > 1) {
    unsigned int current_it = num_vertex - tree;
    for (unsigned int i = current_it; i < num_vertex - 1; i++) {
      // We create the line object within the frame node so that we can
      // set its position and direction relative to its header frame.
      line_[counter].reset(new whole_body_state_rviz_plugin::LineVisual(
          scene_manager_, frame_node_));

      line_[counter]->setArrow(vertices[current_it], vertices[i + 1]);

      counter++;
    }
    tree--;
  }

  // Visualization of the mesh
  if (num_vertex >= 3) {
    mesh_->clear();
    mesh_->estimateVertexCount(num_vertex);
    mesh_->beginTriangles();

    // Adding the vertices
    Ogre::Vector3 normal(0., 0., 1.);
    for (unsigned int i = 0; i < num_vertex; ++i)
      mesh_->addVertex(vertices[i], normal);

    // Adding the actual triangle
    for (unsigned int i = 0; i < num_vertex; i++) {
      mesh_->addTriangle(i % num_vertex, (i + 1) % num_vertex,
                         (i + 2) % num_vertex);
    }
    mesh_->endTriangles();
  }
}

void PolygonVisual::setFramePosition(const Ogre::Vector3 &position) {
  frame_node_->setPosition(position);
  mesh_->setPosition(position);
}

void PolygonVisual::setFrameOrientation(const Ogre::Quaternion &orientation) {
  frame_node_->setOrientation(orientation);
  mesh_->setOrientation(orientation);
}

void PolygonVisual::setLineColor(float r, float g, float b, float a) {
  unsigned int num_line = line_.size();
  for (unsigned int i = 0; i < num_line; ++i) {
    line_[i]->setColor(r, g, b, a);
  }
}

void PolygonVisual::setMeshColor(float r, float g, float b, float a) {
  mesh_->setColor(r, g, b, a);
}

void PolygonVisual::setLineRadius(float radius) {
  unsigned int num_line = line_.size();
  for (unsigned int i = 0; i < num_line; ++i) {
    line_[i]->setProperties(radius);
  }
}

} // namespace whole_body_state_rviz_plugin
