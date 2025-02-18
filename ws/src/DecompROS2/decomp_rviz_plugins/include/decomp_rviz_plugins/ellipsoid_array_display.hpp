#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreQuaternion.h>
#include <OgreVector3.h>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
// #include <rviz_common/frame_manager.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/load_resource.hpp>
#include <rviz_common/message_filter_display.hpp>

#include <decomp_ros_msgs/msg/ellipsoid_array.hpp>
#include "ellipsoid_array_visual.hpp"

#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

namespace rviz_plugins {

class EllipsoidArrayVisual;

class EllipsoidArrayDisplay
  :public rviz_common::RosTopicDisplay<decomp_ros_msgs::msg::EllipsoidArray> {
public:
  EllipsoidArrayDisplay();
  ~EllipsoidArrayDisplay();
  // void onInitialize() override;
  void load(const rviz_common::Config & config) override;
  // void update(float wall_dt, float ros_dt) override;
  // void reset() override;

protected:
  void onInitialize();

  void reset();

private Q_SLOTS:
  void updateColorAndAlpha();

private:

  std::shared_ptr<EllipsoidArrayVisual> visual_;

  rviz_common::properties::ColorProperty *color_property_;
  rviz_common::properties::FloatProperty *alpha_property_;
  void processMessage(const decomp_ros_msgs::msg::EllipsoidArray::ConstSharedPtr msg);
};
}
