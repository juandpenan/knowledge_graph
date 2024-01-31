#include <iostream>
#include <stdexcept>
#include <vector>
#include <algorithm>

#include "knowledge_graph/knowledge_graph.hpp"

namespace knowledge_graph
{
bool PublishTransforms(const knowledge_graph_interfaces::msg::SceneGraph & graph,
 std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster,
 const std_msgs::msg::Header& header)
{
    // Check if both rooms and objects vectors are not empty
    if (graph.rooms.empty() && graph.objects.empty()) {
        // Both vectors are empty, nothing to publish
        return false;
    }
 // Iterate through rooms and objects in the SceneGraph
    for (const auto& room : graph.rooms) {
        // Create a transform for each room
        geometry_msgs::msg::TransformStamped room_transform;
        room_transform.header = header;
        room_transform.child_frame_id = room.scene_category + "_" + std::to_string(room.id);
        room_transform.transform.translation.x = room.location.x;
        room_transform.transform.translation.y = room.location.y;
        room_transform.transform.translation.z = room.location.z;
        room_transform.transform.rotation.w = 1.0;  // No rotation for rooms

        // Publish the room transform
        tf_static_broadcaster->sendTransform(room_transform);

        for (const auto& object : graph.objects) {
        // Create a transform for each object
        if (object.parent_room == room.id) {
            geometry_msgs::msg::TransformStamped object_transform;
            object_transform.header = header;
            object_transform.header.frame_id = room.scene_category + "_" + std::to_string(room.id);
            object_transform.child_frame_id = object.object_class + "_" + std::to_string(object.id);
            object_transform.transform.translation.x = object.location.x;
            object_transform.transform.translation.y = object.location.y;
            object_transform.transform.translation.z = object.location.z;
            object_transform.transform.rotation.w = 1.0;  // No rotation for objects

            // Publish the object transform
            tf_static_broadcaster->sendTransform(object_transform);
        }
     }
    }

    return true;
}

} // namespace knowledge_graph
