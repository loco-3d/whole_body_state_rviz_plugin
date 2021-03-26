///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2020-2021, University of Edinburgh, Istituto Italiano di
// Tecnologia, University of Oxford.
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef WHOLE_BODY_STATE_RVIZ_PLUGIN_WHOLE_BODY_STATE_DISPLAY_H
#define WHOLE_BODY_STATE_RVIZ_PLUGIN_WHOLE_BODY_STATE_DISPLAY_H

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <rviz/message_filter_display.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/robot/robot.h>
#include <whole_body_state_msgs/WholeBodyState.h>

#include "whole_body_state_rviz_plugin/ArrowVisual.h"
#include "whole_body_state_rviz_plugin/PointVisual.h"
#include "whole_body_state_rviz_plugin/PolygonVisual.h"

namespace Ogre {
class SceneNode;
}

namespace rviz {
class BoolProperty;
class EnumProperty;
class ColorProperty;
class FloatProperty;
class IntProperty;
class Shape;
} // namespace rviz

namespace whole_body_state_rviz_plugin {

/**
 * @class WholeBodyStateDisplay
 * @brief Displays a whole_body_state_msgs::WholeBodyState message
 */
class WholeBodyStateDisplay
    : public rviz::MessageFilterDisplay<whole_body_state_msgs::WholeBodyState> {
  Q_OBJECT
public:
  /** @brief Constructor function */
  WholeBodyStateDisplay();

  /** @brief Destructor function */
  ~WholeBodyStateDisplay();

  /** @brief Initialization procedure of the plugin. */
  void onInitialize() override;

  /** @brief Enable procedure of the plugin. */
  void onEnable() override;

  /** @brief Disable procedure of the plugin. */
  void onDisable() override;

  /** @brief Called when the fixed frame changed */
  void fixedFrameChanged() override;

  /** @brief Clear the visuals by deleting their objects */
  void reset() override;

  /**
   * @brief Function to handle an incoming ROS message
   * This is our callback to handle an incoming message
   * @param const whole_body_state_msgs::WholeBodyState::ConstPtr& Whole-body
   * state msg
   */
  void processMessage(
      const whole_body_state_msgs::WholeBodyState::ConstPtr &msg) override;

  /** @brief render callback */
  void update(float wall_dt, float ros_dt) override;

private Q_SLOTS:
  /**@{*/
  /** Helper function to apply color and alpha to all visuals.
   * Set the current color and alpha values for each visual */
  void updateRobotEnable();
  void updateRobotModel();
  void updateRobotVisualVisible();
  void updateRobotCollisionVisible();
  void updateRobotAlpha();
  void updateCoMEnable();
  void updateCoMStyle();
  void updateCoMColorAndAlpha();
  void updateCoMArrowGeometry();
  void updateCoPEnable();
  void updateCoPColorAndAlpha();
  void updateICPEnable();
  void updateICPColorAndAlpha();
  void updateCMPEnable();
  void updateCMPColorAndAlpha();
  void updateGRFEnable();
  void updateGRFColorAndAlpha();
  void updateGRFArrowGeometry();
  void updateSupportEnable();
  void updateSupportLineColorAndAlpha();
  void updateSupportMeshColorAndAlpha();
  void updateFrictionConeEnable();
  void updateFrictionConeColorAndAlpha();
  void updateFrictionConeGeometry();
  /**@}*/

private:
  void processWholeBodyState();
  bool has_new_msg_ = false; ///< Callback sets this to tell our update function
                             ///< it needs to update the model

  /** @brief Loads a URDF from the ros-param named by our
   * "Robot Description" property, iterates through the links, and
   * loads any necessary models.
   */
  void loadRobotModel();

  /** @brief Clear the robot model */
  void clearRobotModel();

  /** @brief Whole-body state message */
  whole_body_state_msgs::WholeBodyState::ConstPtr msg_;

  /**@{*/
  /** Properties to show on side panel */
  rviz::Property *robot_category_;
  rviz::Property *com_category_;
  rviz::Property *cop_category_;
  rviz::Property *cmp_category_;
  rviz::Property *icp_category_;
  rviz::Property *grf_category_;
  rviz::Property *support_category_;
  rviz::Property *friction_category_;
  /**@}*/

  /**@{*/
  /** Object for visualization of the data */
  boost::shared_ptr<rviz::Robot> robot_;
  boost::shared_ptr<PointVisual> com_visual_;
  boost::shared_ptr<ArrowVisual> comd_visual_;
  boost::shared_ptr<PointVisual> cop_visual_;
  boost::shared_ptr<PointVisual> cmp_visual_;
  boost::shared_ptr<PointVisual> icp_visual_;
  std::vector<boost::shared_ptr<ArrowVisual>> grf_visual_;
  boost::shared_ptr<PolygonVisual> support_visual_;
  std::vector<boost::shared_ptr<rviz::Shape>> cones_visual_;
  /**@}*/

  /**@{*/
  /** Property objects for user-editable properties */
  rviz::BoolProperty *robot_enable_property_;
  rviz::StringProperty *robot_model_property_;
  rviz::Property *robot_visual_enabled_property_;
  rviz::Property *robot_collision_enabled_property_;
  rviz::FloatProperty *robot_alpha_property_;
  rviz::BoolProperty *com_enable_property_;
  rviz::EnumProperty *com_style_property_;
  rviz::ColorProperty *com_color_property_;
  rviz::FloatProperty *com_alpha_property_;
  rviz::FloatProperty *com_radius_property_;
  rviz::FloatProperty *com_head_radius_property_;
  rviz::FloatProperty *com_head_length_property_;
  rviz::FloatProperty *com_shaft_radius_property_;
  rviz::FloatProperty *com_shaft_length_property_;
  rviz::BoolProperty *cop_enable_property_;
  rviz::ColorProperty *cop_color_property_;
  rviz::FloatProperty *cop_alpha_property_;
  rviz::FloatProperty *cop_radius_property_;
  rviz::BoolProperty *icp_enable_property_;
  rviz::ColorProperty *icp_color_property_;
  rviz::FloatProperty *icp_alpha_property_;
  rviz::FloatProperty *icp_radius_property_;
  rviz::BoolProperty *cmp_enable_property_;
  rviz::ColorProperty *cmp_color_property_;
  rviz::FloatProperty *cmp_alpha_property_;
  rviz::FloatProperty *cmp_radius_property_;
  rviz::BoolProperty *grf_enable_property_;
  rviz::ColorProperty *grf_color_property_;
  rviz::FloatProperty *grf_alpha_property_;
  rviz::FloatProperty *grf_head_radius_property_;
  rviz::FloatProperty *grf_head_length_property_;
  rviz::FloatProperty *grf_shaft_radius_property_;
  rviz::FloatProperty *grf_shaft_length_property_;
  rviz::BoolProperty *support_enable_property_;
  rviz::ColorProperty *support_line_color_property_;
  rviz::FloatProperty *support_line_alpha_property_;
  rviz::FloatProperty *support_line_radius_property_;
  rviz::ColorProperty *support_mesh_color_property_;
  rviz::FloatProperty *support_mesh_alpha_property_;
  rviz::FloatProperty *support_force_threshold_property_;
  rviz::BoolProperty *friction_cone_enable_property_;
  rviz::ColorProperty *friction_cone_color_property_;
  rviz::FloatProperty *friction_cone_alpha_property_;
  rviz::FloatProperty *friction_cone_length_property_;
  /**@}*/

  /**@{*/
  /** @brief Robot and whole-boyd variables */
  std::string robot_model_;
  bool initialized_model_;
  pinocchio::Model model_;
  pinocchio::Data data_;
  double force_threshold_; //!< Force threshold for detecting active contacts
  double weight_;
  double gravity_;
  double friction_mu_;
  /**@}*/

  enum CoMStyle { REAL, PROJECTED }; //!< CoM visualization style
  bool com_real_; //!< Label to indicates the type of CoM display (real or
                  //!< projected)

  /**@{*/
  /** Flag that indicates if the category are enable */
  bool robot_enable_;
  bool com_enable_;
  bool cop_enable_;
  bool icp_enable_;
  bool cmp_enable_;
  bool grf_enable_;
  bool support_enable_;
  bool cone_enable_;
  /**@}*/
};

} // namespace whole_body_state_rviz_plugin

#endif // WHOLE_BODY_STATE_RVIZ_PLUGIN_WHOLE_BODY_STATE_DISPLAY_H
