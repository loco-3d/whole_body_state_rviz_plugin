#ifndef STATE_RVIZ_PLUGIN_WHOLE_BODY_STATE_DISPLAY_H
#define STATE_RVIZ_PLUGIN_WHOLE_BODY_STATE_DISPLAY_H

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#ifndef Q_MOC_RUN
#include <boost/circular_buffer.hpp>
#endif

#include <rviz/message_filter_display.h>

#include "state_rviz_plugin/ArrowVisual.h"
#include "state_rviz_plugin/PointVisual.h"
#include "state_rviz_plugin/PolygonVisual.h"
#include <state_msgs/WholeBodyState.h>

namespace Ogre {
class SceneNode;
}

namespace rviz {
class EnumProperty;
class ColorProperty;
class FloatProperty;
class IntProperty;
class Shape;
} // namespace rviz

namespace state_rviz_plugin {

/**
 * @class WholeBodyStateDisplay
 * @brief Displays a dwl_msgs::WholeBodyState message
 */
class WholeBodyStateDisplay
    : public rviz::MessageFilterDisplay<state_msgs::WholeBodyState> {
  Q_OBJECT
public:
  /** @brief Constructor function */
  WholeBodyStateDisplay();

  /** @brief Destructor function */
  ~WholeBodyStateDisplay();

  /** @brief Clear the robot model */
  void clear();

  /** @brief Overrides of public virtual functions from the Display class */
  void onInitialize();
  void onEnable();
  void onDisable();
  void fixedFrameChanged();

  /** @brief Clear the visuals by deleting their objects */
  void reset();

  /** @brief Loads a URDF from the ros-param named by our
   * "Robot Description" property, iterates through the links, and
   * loads any necessary models. */
  void load();

  /**
   * @brief Function to handle an incoming ROS message
   * This is our callback to handle an incoming message
   * @param const state_msgs::WholeBodyState::ConstPtr& Whole-body state msg
   */
  void processMessage(const state_msgs::WholeBodyState::ConstPtr &msg);

private Q_SLOTS:
  /** @brief Helper function to apply color and alpha to all visuals.
   * Set the current color and alpha values for each visual */
  void updateRobotModel();
  void updateCoMStyle();
  void updateCoMColorAndAlpha();
  void updateCoMArrowGeometry();
  void updateCoPColorAndAlpha();
  void updateICPColorAndAlpha();
  void updateCMPColorAndAlpha();
  void updateGRFColorAndAlpha();
  void updateGRFArrowGeometry();
  void updateSupportLineColorAndAlpha();
  void updateSupportMeshColorAndAlpha();
  void updateFrictionConeColorAndAlpha();
  void updateFrictionConeGeometry();

private:
  void processWholeBodyState();

  /** @brief Whole-body state message */
  state_msgs::WholeBodyState::ConstPtr msg_;
  bool is_info_;

  /** @brief Robot URDF model */
  std::string robot_model_;
  bool initialized_model_;

  /** @brief Properties to show on side panel */
  rviz::Property *com_category_;
  rviz::Property *cop_category_;
  rviz::Property *cmp_category_;
  rviz::Property *icp_category_;
  rviz::Property *grf_category_;
  rviz::Property *support_category_;
  rviz::Property *friction_category_;

  /** @brief Object for visualization of the data */
  boost::shared_ptr<PointVisual> com_visual_;
  boost::shared_ptr<ArrowVisual> comd_visual_;
  boost::shared_ptr<PointVisual> cop_visual_;
  boost::shared_ptr<PointVisual> cmp_visual_;
  boost::shared_ptr<PointVisual> icp_visual_;
  std::vector<boost::shared_ptr<ArrowVisual>> grf_visual_;
  boost::shared_ptr<PolygonVisual> support_visual_;

  /** @brief Property objects for user-editable properties */
  rviz::StringProperty *robot_model_property_;
  rviz::EnumProperty *com_style_property_;
  rviz::ColorProperty *com_color_property_;
  rviz::FloatProperty *com_alpha_property_;
  rviz::FloatProperty *com_radius_property_;
  rviz::FloatProperty *com_head_radius_property_;
  rviz::FloatProperty *com_head_length_property_;
  rviz::FloatProperty *com_shaft_radius_property_;
  rviz::FloatProperty *com_shaft_length_property_;

  rviz::ColorProperty *cop_color_property_;
  rviz::FloatProperty *cop_alpha_property_;
  rviz::FloatProperty *cop_radius_property_;

  rviz::ColorProperty *icp_color_property_;
  rviz::FloatProperty *icp_alpha_property_;
  rviz::FloatProperty *icp_radius_property_;

  rviz::ColorProperty *cmp_color_property_;
  rviz::FloatProperty *cmp_alpha_property_;
  rviz::FloatProperty *cmp_radius_property_;

  rviz::ColorProperty *grf_color_property_;
  rviz::FloatProperty *grf_alpha_property_;
  rviz::FloatProperty *grf_head_radius_property_;
  rviz::FloatProperty *grf_head_length_property_;
  rviz::FloatProperty *grf_shaft_radius_property_;
  rviz::FloatProperty *grf_shaft_length_property_;

  rviz::ColorProperty *support_line_color_property_;
  rviz::FloatProperty *support_line_alpha_property_;
  rviz::FloatProperty *support_line_radius_property_;
  rviz::ColorProperty *support_mesh_color_property_;
  rviz::FloatProperty *support_mesh_alpha_property_;
  rviz::FloatProperty *support_force_threshold_property_;

  rviz::ColorProperty *friction_cone_color_property_;
  rviz::FloatProperty *friction_cone_alpha_property_;
  rviz::FloatProperty *friction_cone_length_property_;

  /** @brief Whole-body dynamics */
  pinocchio::Model model_;

  std::vector<boost::shared_ptr<rviz::Shape>>
      cones_visual_;       //!< Handles actually drawing the cones
  double force_threshold_; //!< Force threshold for detecting active contacts
  double weight_;          //!< Weight of the robot
  double gravity_;         //!< Gravity acceleration
  double friction_mu_;     //!< Friction coefficient
  enum CoMStyle { REAL, PROJECTED }; //!< CoM visualization style
  bool com_real_;
};

} // namespace state_rviz_plugin

#endif // STATE_RVIZ_PLUGIN_WHOLE_BODY_STATE_DISPLAY_H
