#ifndef open_ptrack_skeleton_msgs_types_h
#define open_ptrack_skeleton_msgs_types_h

// ROS dependencies
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"

// Standard dependencies
#include <iostream>
#include <limits>
#include <vector>

namespace open_ptrack {
  namespace skeletons {
    namespace types {

      // Vector3
      typedef tf2::Vector3 Vector3;

      std::ostream& operator<<(std::ostream& t_os, const Vector3& t_v);

      // Point
      typedef Vector3 Point;

      // Quaternion
      typedef tf2::Quaternion Quaternion;

      std::ostream& operator<<(std::ostream& t_os, const Quaternion& t_q);

      // Pose
      struct Pose
      {
        Pose(
          const Point& t_position = Point(std::numeric_limits<double>::quiet_NaN(),
                                          std::numeric_limits<double>::quiet_NaN(),
                                          std::numeric_limits<double>::quiet_NaN()),
          const Quaternion& t_orientation = Quaternion(std::numeric_limits<double>::quiet_NaN(),
                                                       std::numeric_limits<double>::quiet_NaN(),
                                                       std::numeric_limits<double>::quiet_NaN(),
                                                       std::numeric_limits<double>::quiet_NaN()));
        Pose(const Quaternion& t_orientation);

        friend std::ostream& operator<<(std::ostream& t_os, const Pose& t_p);

        Point position;
        Quaternion orientation;
      };

      // Velocity
      struct Velocity
      {
        Velocity(const Vector3& t_linear = Vector3(std::numeric_limits<double>::quiet_NaN(),
                                                   std::numeric_limits<double>::quiet_NaN(),
                                                   std::numeric_limits<double>::quiet_NaN()),
                 const Vector3& t_angular = Vector3(std::numeric_limits<double>::quiet_NaN(),
                                                    std::numeric_limits<double>::quiet_NaN(),
                                                    std::numeric_limits<double>::quiet_NaN()));

        friend std::ostream& operator<<(std::ostream& t_os, const Velocity& t_v);

        Vector3 linear;
        Vector3 angular;
      };

      // Acceleration
      typedef Velocity Acceleration;

      // KinematicState
      struct KinematicState
      {
        KinematicState(const Pose& t_pose = Pose(),
                       const Velocity& t_velocity = Velocity(),
                       const Acceleration& t_acceleration = Acceleration());
        KinematicState(const Point& t_position);
        KinematicState(const Quaternion& t_orientation);

        friend std::ostream& operator<<(std::ostream& t_os, const KinematicState& t_ks);

        Pose pose;
        Velocity velocity;
        Acceleration acceleration;
      };

      // Box
      struct Box
      {
        Box(const KinematicState& t_center = KinematicState(),
            const double& t_height = std::numeric_limits<double>::quiet_NaN(),
            const double& t_length = std::numeric_limits<double>::quiet_NaN(),
            const double& t_width = std::numeric_limits<double>::quiet_NaN());

        friend std::ostream& operator<<(std::ostream& t_os, const Box& t_b);

        KinematicState center;
        double height;
        double length;
        double width;
      };

      // Marker
      struct Marker
      {
        Marker(const int& t_id = -1,
               const std::string& t_name = "",
               const double& t_confidence = std::numeric_limits<double>::quiet_NaN(),
               const KinematicState& t_center = KinematicState());
        Marker(const int& t_id,
               const double& t_confidence = std::numeric_limits<double>::quiet_NaN(),
               const KinematicState& t_center = KinematicState());

        friend std::ostream& operator<<(std::ostream& t_os, const Marker& t_m);

        int id;
        std::string name;
        double confidence;
        KinematicState center;
      };

      // Link
      struct Link
      {
        Link(const int& t_id = -1,
             const std::string& t_name = "",
             const int& t_parent_marker = -1,
             const int& t_child_marker = -1,
             const double& t_confidence = std::numeric_limits<double>::quiet_NaN(),
             const KinematicState& t_center = KinematicState());
        Link(const int& t_id,
             const int& t_parent_marker = -1,
             const int& t_child_marker = -1,
             const double& t_confidence = std::numeric_limits<double>::quiet_NaN(),
             const KinematicState& t_center = KinematicState());

        friend std::ostream& operator<<(std::ostream& t_os, const Link& t_l);

        int id;
        std::string name;
        int parent_marker;
        int child_marker;
        double confidence;
        KinematicState center;
      };

      // Skeleton
      struct Skeleton
      {
        Skeleton(const int& t_id = -1,
                 const double& t_src_time = std::numeric_limits<double>::quiet_NaN(),
                 const std::string t_src_frame = "",
                 const unsigned int& t_max_markers = 0,
                 const unsigned int& t_max_links = 0,
                 const double& t_confidence = std::numeric_limits<double>::quiet_NaN(),
                 const Box& t_bounding_box = Box(),
                 const std::vector<Marker>& t_markers = std::vector<Marker>(),
                 const std::vector<Link>& t_links = std::vector<Link>());

        const Marker& getMarker(const int& t_id) const;
        Marker& getMarker(const int& t_id);
        bool hasMarker(const int& t_id) const;
        bool addMarker(const Marker& t_marker);
        bool removeMarker(const int& t_id);

        const Link& getLink(const int& t_id) const;
        Link& getLink(const int& t_id);
        bool hasLink(const int& t_id) const;
        bool addLink(const Link& t_link);
        bool removeLink(const int& t_id);

        friend std::ostream& operator<<(std::ostream& t_os, const Skeleton& t_s);

        int id;
        double src_time;
        std::string src_frame;
        unsigned int max_markers;
        unsigned int max_links;
        double confidence;
        Box bounding_box;
        std::vector<Marker> markers;
        std::vector<Link> links;
      };

      // SkeletonGroup
      struct SkeletonGroup
      {
        SkeletonGroup(const double& t_time = std::numeric_limits<double>::quiet_NaN(),
                      const std::string& t_frame = "",
                      const std::vector<Skeleton>& t_skeletons = std::vector<Skeleton>());

        const Skeleton& getSkeleton(const int& t_id) const;
        Skeleton& getSkeleton(const int& t_id);
        bool hasSkeleton(const int& t_id) const;
        bool addSkeleton(const Skeleton& t_skeleton);
        bool removeSkeleton(const int& t_id);

        friend std::ostream& operator<<(std::ostream& t_os, const SkeletonGroup& t_sg);

        double time;
        std::string frame;
        std::vector<Skeleton> skeletons;
      };

    } // namespace types
  } // namespace skeletons
} // namespace open_ptrack

#endif
