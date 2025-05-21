#include "tf_lcm/transform_storage.hpp"
#include <ctime>

namespace tf_lcm {

TransformStorage::TransformStorage()
    : frame_id(""),
      child_frame_id(""),
      stamp(std::chrono::system_clock::now()),
      authority(""),
      translation_x(0.0),
      translation_y(0.0),
      translation_z(0.0),
      rotation_x(0.0),
      rotation_y(0.0),
      rotation_z(0.0),
      rotation_w(1.0) {
}

TransformStorage::TransformStorage(const geometry_msgs::TransformStamped& transform, const std::string& auth)
    : frame_id(transform.header.frame_id),
      child_frame_id(transform.child_frame_id),
      authority(auth) {
    
    // Convert ROS time to system_clock time
    auto seconds = transform.header.stamp.sec;
    auto nanoseconds = transform.header.stamp.nsec;
    
    // Create a time_point from the seconds and nanoseconds
    // Convert to a system_clock::time_point
    std::time_t sec_time = static_cast<std::time_t>(seconds);
    stamp = std::chrono::system_clock::from_time_t(sec_time);
    
    // Add nanoseconds by using duration_cast to ensure compatibility
    stamp += std::chrono::duration_cast<std::chrono::system_clock::duration>(
        std::chrono::nanoseconds(nanoseconds));

    // Store transform data
    translation_x = transform.transform.translation.x;
    translation_y = transform.transform.translation.y;
    translation_z = transform.transform.translation.z;
    rotation_x = transform.transform.rotation.x;
    rotation_y = transform.transform.rotation.y;
    rotation_z = transform.transform.rotation.z;
    rotation_w = transform.transform.rotation.w;
}

geometry_msgs::TransformStamped TransformStorage::toTransformStamped() const {
    geometry_msgs::TransformStamped result;
    
    // Set header
    result.header.frame_id = frame_id;
    
    // Convert system_clock time to ROS time
    auto seconds = std::chrono::time_point_cast<std::chrono::seconds>(stamp);
    auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(stamp - seconds);
    
    result.header.stamp.sec = static_cast<int64_t>(
        std::chrono::duration_cast<std::chrono::seconds>(seconds.time_since_epoch()).count());
    result.header.stamp.nsec = static_cast<int64_t>(nanoseconds.count());
    
    // Set child frame and transform
    result.child_frame_id = child_frame_id;
    
    result.transform.translation.x = translation_x;
    result.transform.translation.y = translation_y;
    result.transform.translation.z = translation_z;
    
    result.transform.rotation.x = rotation_x;
    result.transform.rotation.y = rotation_y;
    result.transform.rotation.z = rotation_z;
    result.transform.rotation.w = rotation_w;
    
    return result;
}

} // namespace tf_lcm
