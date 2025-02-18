#include <decomp_rviz_plugins/ellipsoid_array_display.hpp>

namespace rviz_plugins {

EllipsoidArrayDisplay::EllipsoidArrayDisplay() {
  color_property_ = new rviz_common::properties::ColorProperty("Color", QColor(204, 51, 204),
                                            "Color of ellipsoids.",
                                            this, SLOT(updateColorAndAlpha()));
  alpha_property_ = new rviz_common::properties::FloatProperty(
      "Alpha", 0.5, "0 is fully transparent, 1.0 is fully opaque.", this,
      SLOT(updateColorAndAlpha()));

}

void EllipsoidArrayDisplay::onInitialize() {
  RTDClass::onInitialize();
}

EllipsoidArrayDisplay::~EllipsoidArrayDisplay() {}

void EllipsoidArrayDisplay::reset() {
  visual_ = nullptr;
}

void EllipsoidArrayDisplay::updateColorAndAlpha() {
  float alpha = alpha_property_->getFloat();
  Ogre::ColourValue color = color_property_->getOgreColor();

  if (visual_)
    visual_->setColor(color.r, color.g, color.b, alpha);
}

void EllipsoidArrayDisplay::processMessage(const decomp_ros_msgs::msg::EllipsoidArray::ConstSharedPtr msg) {
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if (!context_->getFrameManager()->getTransform(
          msg->header.frame_id, msg->header.stamp, position, orientation)) {
    // ROS_DEBUG("Error transforming from frame '%s' to frame '%s'",
    //           msg->header.frame_id.c_str(), qPrintable(fixed_frame_));
    return;
  }

  std::shared_ptr<EllipsoidArrayVisual> visual;
  visual.reset(new EllipsoidArrayVisual(context_->getSceneManager(), scene_node_));

  visual->setMessage(msg);
  visual->setFramePosition(position);
  visual->setFrameOrientation(orientation);

  float alpha = alpha_property_->getFloat();
  Ogre::ColourValue color = color_property_->getOgreColor();
  visual->setColor(color.r, color.g, color.b, alpha);

  visual_ = visual;
  return;
}

void EllipsoidArrayDisplay::load(const rviz_common::Config & config)
{
  rviz_common::Display::load(config);
}

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::EllipsoidArrayDisplay, rviz_common::Display)
