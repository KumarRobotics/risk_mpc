#include <decomp_ros_msgs/msg/polyhedron_array.hpp>
#include <rviz_common/message_filter_display.hpp>

#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/visualization_manager.hpp>
// #include <rviz_common/frame_manager.hpp>
#include <rviz_common/frame_manager_iface.hpp>

#include <OgreQuaternion.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreVector3.h>

#include <rviz_common/load_resource.hpp>

#include "rviz_common/ros_topic_display.hpp"
#include <decomp_ros_util/data_ros_utils.hpp>
#include <decomp_geometry/geometric_utils.h>
#include <decomp_rviz_plugins/bound_visual.hpp>
#include <decomp_rviz_plugins/mesh_visual.hpp>
#include <decomp_rviz_plugins/vector_visual.hpp>

namespace rviz_plugins {
    class PolyhedronArrayDisplay
        : public rviz_common::RosTopicDisplay<decomp_ros_msgs::msg::PolyhedronArray> {
    public:
        PolyhedronArrayDisplay();
        virtual ~PolyhedronArrayDisplay();

    protected:
        virtual void onInitialize();

        virtual void reset();

    private Q_SLOTS:
        void updateMeshColorAndAlpha();
        void updateBoundColorAndAlpha();
        void updateVsColorAndAlpha();
        void updateState();
        void updateScale();
        void updateVsScale();

    private:
        void visualizeMessage(int state);
        void visualizeMesh();
        void processMessage(const decomp_ros_msgs::msg::PolyhedronArray::ConstSharedPtr msg);
        void visualizeBound();
        void visualizeVs();

        std::shared_ptr<MeshVisual> visual_mesh_;
        std::shared_ptr<BoundVisual> visual_bound_;
        std::shared_ptr<VectorVisual> visual_vector_;

        rviz_common::properties::ColorProperty *mesh_color_property_;
        rviz_common::properties::ColorProperty *bound_color_property_;
        rviz_common::properties::ColorProperty *vs_color_property_;
        rviz_common::properties::FloatProperty *alpha_property_;
        rviz_common::properties::FloatProperty *scale_property_;
        rviz_common::properties::FloatProperty *vs_scale_property_;
        rviz_common::properties::EnumProperty *state_property_;

        Ogre::Vector3 position_;
        Ogre::Quaternion orientation_;

        vec_E<vec_Vec3f> vertices_;
        vec_E<std::pair<Vec3f, Vec3f>> vs_;
    };

}// namespace rviz_plugins
