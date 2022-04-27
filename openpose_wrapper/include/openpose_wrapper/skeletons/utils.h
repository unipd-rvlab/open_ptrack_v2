#ifndef open_ptrack_skeleton_msgs_utils_h
#define open_ptrack_skeleton_msgs_utils_h

// Ros Distributed Message dependencies
#include <geometry_msgs/Accel.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

// Custom Ros Message dependencies
#include "openpose_wrapper/Box.h"
#include "openpose_wrapper/KinematicState.h"
#include "openpose_wrapper/Link.h"
#include "openpose_wrapper/Marker.h"
#include "openpose_wrapper/Skeleton.h"
#include "openpose_wrapper/SkeletonGroup.h"

// Internal dependencies
#include "openpose_wrapper/skeletons/types.h"

namespace open_ptrack {
  namespace skeletons {
    namespace utils {

      const std::string padding(int t_n_pads);

      // Vector3
      open_ptrack::skeletons::types::Vector3
      toStruct(const double& t_x,
               const double& t_y,
               const double& t_z = std::numeric_limits<double>::quiet_NaN());

      open_ptrack::skeletons::types::Vector3 toStruct(const geometry_msgs::Vector3& t_v);

      geometry_msgs::Vector3 toVector3Msg(const open_ptrack::skeletons::types::Vector3& t_v);

      bool isNaN(const open_ptrack::skeletons::types::Vector3& t_v);

      double magnitude(const open_ptrack::skeletons::types::Vector3& t_v);

      std::string toString(const open_ptrack::skeletons::types::Vector3& t_v, int t_pad_lv = 0);

      // Point
      open_ptrack::skeletons::types::Point toStruct(const geometry_msgs::Point& t_p);

      geometry_msgs::Point toPointMsg(const open_ptrack::skeletons::types::Point& t_p);

      double distance(const open_ptrack::skeletons::types::Point& t_p1,
                      const open_ptrack::skeletons::types::Point& t_p2);

      // Quaternion
      open_ptrack::skeletons::types::Quaternion
      toStruct(const double& t_x, const double& t_y, const double& t_z, const double& t_w);

      open_ptrack::skeletons::types::Quaternion toStruct(const geometry_msgs::Quaternion& t_q);

      geometry_msgs::Quaternion toMsg(const open_ptrack::skeletons::types::Quaternion& t_q);

      bool isNaN(const open_ptrack::skeletons::types::Quaternion& t_q);

      double distance(const open_ptrack::skeletons::types::Quaternion& t_q1,
                      const open_ptrack::skeletons::types::Quaternion& t_q2);

      std::string toString(const open_ptrack::skeletons::types::Quaternion& t_q, int t_pad_lv = 0);

      // Pose
      open_ptrack::skeletons::types::Pose
      toStruct(const open_ptrack::skeletons::types::Point& t_position,
               const open_ptrack::skeletons::types::Quaternion& t_orientation =
                 open_ptrack::skeletons::types::Quaternion(std::numeric_limits<double>::quiet_NaN(),
                                                     std::numeric_limits<double>::quiet_NaN(),
                                                     std::numeric_limits<double>::quiet_NaN(),
                                                     std::numeric_limits<double>::quiet_NaN()));
      open_ptrack::skeletons::types::Pose
      toStruct(const open_ptrack::skeletons::types::Quaternion& t_orientation);

      open_ptrack::skeletons::types::Pose toStruct(const geometry_msgs::Pose& t_p);

      geometry_msgs::Pose toMsg(const open_ptrack::skeletons::types::Pose& t_p);

      bool isNaN(const open_ptrack::skeletons::types::Pose& t_p);

      std::string toString(const open_ptrack::skeletons::types::Pose& t_p, int t_pad_lv = 0);

      // Velocity
      open_ptrack::skeletons::types::Velocity
      toStruct(const open_ptrack::skeletons::types::Vector3& t_linear,
               const open_ptrack::skeletons::types::Vector3& t_angular =
                 open_ptrack::skeletons::types::Vector3(std::numeric_limits<double>::quiet_NaN(),
                                                  std::numeric_limits<double>::quiet_NaN(),
                                                  std::numeric_limits<double>::quiet_NaN()));

      open_ptrack::skeletons::types::Velocity toStruct(const geometry_msgs::Twist& t_v);

      geometry_msgs::Twist toTwistMsg(const open_ptrack::skeletons::types::Velocity& t_v);

      bool isNaN(const open_ptrack::skeletons::types::Velocity& t_v);

      std::string toString(const open_ptrack::skeletons::types::Velocity& t_v, int t_pad_lv = 0);

      // Acceleration
      open_ptrack::skeletons::types::Acceleration toStruct(const geometry_msgs::Accel& t_a);

      geometry_msgs::Accel toAccelMsg(const open_ptrack::skeletons::types::Acceleration& t_a);

      // KinematicState
      open_ptrack::skeletons::types::KinematicState toStruct(
        const open_ptrack::skeletons::types::Pose& t_pose,
        const open_ptrack::skeletons::types::Velocity& t_velocity = open_ptrack::skeletons::types::Velocity(),
        const open_ptrack::skeletons::types::Acceleration& t_acceleration =
          open_ptrack::skeletons::types::Acceleration());

      open_ptrack::skeletons::types::KinematicState
      toStruct(const openpose_wrapper::KinematicState& t_ks);

      openpose_wrapper::KinematicState
      toMsg(const open_ptrack::skeletons::types::KinematicState& t_ks);

      std::string toString(const open_ptrack::skeletons::types::KinematicState& t_ks, int t_pad_lv = 0);

      // Box
      open_ptrack::skeletons::types::Box
      toStruct(const open_ptrack::skeletons::types::KinematicState& t_center,
               const double& t_height,
               const double& t_length,
               const double& t_width = std::numeric_limits<double>::quiet_NaN());

      open_ptrack::skeletons::types::Box toStruct(const openpose_wrapper::Box& t_b);

      openpose_wrapper::Box toMsg(const open_ptrack::skeletons::types::Box& t_b);

      std::string toString(const open_ptrack::skeletons::types::Box& t_b, int t_pad_lv = 0);

      // Marker
      open_ptrack::skeletons::types::Marker
      toStruct(const int& t_id,
               const std::string& t_name,
               const double& t_confidence,
               const open_ptrack::skeletons::types::KinematicState& t_center);

      open_ptrack::skeletons::types::Marker toStruct(const openpose_wrapper::Marker& t_m);

      openpose_wrapper::Marker toMsg(const open_ptrack::skeletons::types::Marker& t_m);

      std::string toString(const open_ptrack::skeletons::types::Marker& t_m, int t_pad_lv = 0);

      // Link
      open_ptrack::skeletons::types::Link
      toStruct(const int& t_id,
               const std::string& t_name,
               const int& t_parent_marker,
               const int& t_child_marker,
               const double& t_confidence,
               const open_ptrack::skeletons::types::KinematicState& t_center);

      open_ptrack::skeletons::types::Link toStruct(const openpose_wrapper::Link& t_l);

      openpose_wrapper::Link toMsg(const open_ptrack::skeletons::types::Link& t_l);

      std::string toString(const open_ptrack::skeletons::types::Link& t_l, int t_pad_lv = 0);

      // Skeleton
      open_ptrack::skeletons::types::Skeleton
      toStruct(const int& t_id,
               const double& t_src_time,
               const std::string& t_src_frame,
               const unsigned int& t_max_markers = 0,
               const unsigned int& t_max_links = 0,
               const double& t_confidence = std::numeric_limits<double>::quiet_NaN(),
               const open_ptrack::skeletons::types::Box& t_bounding_box = open_ptrack::skeletons::types::Box(),
               const std::vector<open_ptrack::skeletons::types::Marker>& t_markers =
                 std::vector<open_ptrack::skeletons::types::Marker>(),
               const std::vector<open_ptrack::skeletons::types::Link>& t_links =
                 std::vector<open_ptrack::skeletons::types::Link>());

      open_ptrack::skeletons::types::Skeleton toStruct(const openpose_wrapper::Skeleton& t_s);

      openpose_wrapper::Skeleton toMsg(const open_ptrack::skeletons::types::Skeleton& t_s);

      open_ptrack::skeletons::types::Box computeBoundingBox(const open_ptrack::skeletons::types::Skeleton& t_s);

      open_ptrack::skeletons::types::KinematicState
      centroid(const open_ptrack::skeletons::types::Skeleton& t_s);

      std::string toString(const open_ptrack::skeletons::types::Skeleton& t_s, int t_pad_lv = 0);

      // SkeletonGroup
      open_ptrack::skeletons::types::SkeletonGroup
      toStruct(const double& t_time,
               const std::string& t_frame,
               const std::vector<open_ptrack::skeletons::types::Skeleton>& t_skeletons);

      open_ptrack::skeletons::types::SkeletonGroup
      toStruct(const openpose_wrapper::SkeletonGroup& t_sg);

      openpose_wrapper::SkeletonGroup toMsg(const open_ptrack::skeletons::types::SkeletonGroup& t_sg);

      std::string toString(const open_ptrack::skeletons::types::SkeletonGroup& t_sg, int t_pad_lv = 0);

    } // namespace utils
  } // namespace skeletons
} // namespace open_ptrack

#endif
