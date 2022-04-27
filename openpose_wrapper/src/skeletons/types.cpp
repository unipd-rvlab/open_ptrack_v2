// Internal dependencies
#include "openpose_wrapper/skeletons/types.h"
#include "openpose_wrapper/skeletons/utils.h"

namespace open_ptrack {
  namespace skeletons {
    namespace types {

      // Vector3
      std::ostream& operator<<(std::ostream& t_os, const Vector3& t_v)
      {
        return t_os << utils::toString(t_v);
      }

      // Quaternion
      std::ostream& operator<<(std::ostream& t_os, const Quaternion& t_q)
      {
        return t_os << utils::toString(t_q);
      }

      // Pose
      Pose::Pose(const Vector3& t_position, const Quaternion& t_orientation)
        : position(t_position)
        , orientation(t_orientation)
      {}

      Pose::Pose(const Quaternion& t_orientation)
        : Pose(Point(std::numeric_limits<double>::quiet_NaN(),
                     std::numeric_limits<double>::quiet_NaN(),
                     std::numeric_limits<double>::quiet_NaN()),
               t_orientation)
      {}

      std::ostream& operator<<(std::ostream& t_os, const Pose& t_p)
      {
        return t_os << utils::toString(t_p);
      }

      // Velocity
      Velocity::Velocity(const Vector3& t_linear, const Vector3& t_angular)
        : linear(t_linear)
        , angular(t_angular)
      {}

      std::ostream& operator<<(std::ostream& t_os, const Velocity& t_v)
      {
        return t_os << utils::toString(t_v);
      }

      // KinematicState
      KinematicState::KinematicState(const Pose& t_pose,
                                     const Velocity& t_velocity,
                                     const Acceleration& t_acceleration)
        : pose(t_pose)
        , velocity(t_velocity)
        , acceleration(t_acceleration)
      {}

      KinematicState::KinematicState(const Point& t_position)
        : KinematicState(Pose(t_position))
      {}

      KinematicState::KinematicState(const Quaternion& t_orientation)
        : KinematicState(Pose(t_orientation))
      {}

      std::ostream& operator<<(std::ostream& t_os, const KinematicState& t_ks)
      {
        return t_os << utils::toString(t_ks);
      }

      // Box
      Box::Box(const KinematicState& t_center,
               const double& t_height,
               const double& t_length,
               const double& t_width)
        : center(t_center)
        , height(t_height)
        , length(t_length)
        , width(t_width)
      {}

      std::ostream& operator<<(std::ostream& t_os, const Box& t_b)
      {
        return t_os << utils::toString(t_b);
      }

      // Marker
      Marker::Marker(const int& t_id,
                     const std::string& t_name,
                     const double& t_confidence,
                     const KinematicState& t_center)
        : id(t_id)
        , name(t_name)
        , confidence(t_confidence)
        , center(t_center)
      {}

      Marker::Marker(const int& t_id, const double& t_confidence, const KinematicState& t_center)
        : Marker(t_id, "", t_confidence, t_center)
      {}

      std::ostream& operator<<(std::ostream& t_os, const Marker& t_m)
      {
        return t_os << utils::toString(t_m);
      }

      // Link
      Link::Link(const int& t_id,
                 const std::string& t_name,
                 const int& t_parent_marker,
                 const int& t_child_marker,
                 const double& t_confidence,
                 const KinematicState& t_center)
        : id(t_id)
        , name(t_name)
        , parent_marker(t_parent_marker)
        , child_marker(t_child_marker)
        , confidence(t_confidence)
        , center(t_center)
      {}

      Link::Link(const int& t_id,
                 const int& t_parent_marker,
                 const int& t_child_marker,
                 const double& t_confidence,
                 const KinematicState& t_center)
        : Link(t_id, "", t_parent_marker, t_child_marker, t_confidence, t_center)
      {}

      std::ostream& operator<<(std::ostream& t_os, const Link& t_l)
      {
        return t_os << utils::toString(t_l);
      }

      // Skeleton
      Skeleton::Skeleton(const int& t_id,
                         const double& t_src_time,
                         const std::string t_src_frame,
                         const unsigned int& t_max_markers,
                         const unsigned int& t_max_links,
                         const double& t_confidence,
                         const Box& t_bounding_box,
                         const std::vector<Marker>& t_markers,
                         const std::vector<Link>& t_links)
        : id(t_id)
        , src_time(t_src_time)
        , src_frame(t_src_frame)
        , max_markers(t_max_markers)
        , max_links(t_max_links)
        , confidence(t_confidence)
        , bounding_box(t_bounding_box)
        , markers(t_markers)
        , links(t_links)
      {}

      const Marker& Skeleton::getMarker(const int& t_id) const
      {
        for (const auto& marker : markers) {
          if (marker.id == t_id) {
            return marker;
          }
        }
        throw std::out_of_range("Marker " + std::to_string(t_id) + " not present in Skeleton "
                                + std::to_string(id));
      }

      Marker& Skeleton::getMarker(const int& t_id)
      {
        for (auto& marker : markers) {
          if (marker.id == t_id) {
            return marker;
          }
        }
        throw std::out_of_range("Marker " + std::to_string(t_id) + " not present in Skeleton "
                                + std::to_string(id));
      }

      bool Skeleton::hasMarker(const int& t_id) const
      {
        for (const auto& marker : markers) {
          if (marker.id == t_id) {
            return true;
          }
        }
        return false;
      }

      bool Skeleton::addMarker(const Marker& t_marker)
      {
        if (hasMarker(t_marker.id)) {
          return false;
        }

        markers.push_back(t_marker);
        return true;
      }

      bool Skeleton::removeMarker(const int& t_id)
      {
        if (!hasMarker(t_id)) {
          return false;
        }

        markers.erase(std::remove_if(
          markers.begin(), markers.end(), [&](const Marker& m) { return m.id == t_id; }));
        return true;
      }

      const Link& Skeleton::getLink(const int& t_id) const
      {
        for (const auto& link : links) {
          if (link.id == t_id) {
            return link;
          }
        }
        throw std::out_of_range("Link " + std::to_string(t_id) + " not present in Skeleton "
                                + std::to_string(id));
      }

      Link& Skeleton::getLink(const int& t_id)
      {
        for (auto& link : links) {
          if (link.id == t_id) {
            return link;
          }
        }
        throw std::out_of_range("Link " + std::to_string(t_id) + " not present in Skeleton "
                                + std::to_string(id));
      }

      bool Skeleton::hasLink(const int& t_id) const
      {
        for (const auto& link : links) {
          if (link.id == t_id) {
            return true;
          }
        }
        return false;
      }

      bool Skeleton::addLink(const Link& t_link)
      {
        if (hasLink(t_link.id)) {
          return false;
        }

        links.push_back(t_link);
        return true;
      }

      bool Skeleton::removeLink(const int& t_id)
      {
        if (!hasLink(t_id)) {
          return false;
        }

        links.erase(
          std::remove_if(links.begin(), links.end(), [&](const Link& l) { return l.id == t_id; }));
        return true;
      }

      std::ostream& operator<<(std::ostream& t_os, const Skeleton& t_s)
      {
        return t_os << utils::toString(t_s);
      }

      // SkeletonGroup
      SkeletonGroup::SkeletonGroup(const double& t_time,
                                   const std::string& t_frame,
                                   const std::vector<Skeleton>& t_skeletons)
        : time(t_time)
        , frame(t_frame)
        , skeletons(t_skeletons)
      {}

      const Skeleton& SkeletonGroup::getSkeleton(const int& t_id) const
      {
        for (const auto& skeleton : skeletons) {
          if (skeleton.id == t_id) {
            return skeleton;
          }
        }
        throw std::out_of_range("Skeleton " + std::to_string(t_id)
                                + " not present in SkeletonGroup");
      }

      Skeleton& SkeletonGroup::getSkeleton(const int& t_id)
      {
        for (auto& skeleton : skeletons) {
          if (skeleton.id == t_id) {
            return skeleton;
          }
        }
        throw std::out_of_range("Skeleton " + std::to_string(t_id)
                                + " not present in SkeletonGroup");
      }

      bool SkeletonGroup::hasSkeleton(const int& t_id) const
      {
        for (const auto& skeleton : skeletons) {
          if (skeleton.id == t_id) {
            return true;
          }
        }
        return false;
      }

      bool SkeletonGroup::addSkeleton(const Skeleton& t_skeleton)
      {
        if (hasSkeleton(t_skeleton.id)) {
          return false;
        }

        skeletons.push_back(t_skeleton);
        return true;
      }

      bool SkeletonGroup::removeSkeleton(const int& t_id)
      {
        if (!hasSkeleton(t_id)) {
          return false;
        }

        skeletons.erase(std::remove_if(skeletons.begin(),
                                       skeletons.end(),
                                       [&](const Skeleton& s) { return (s.id == t_id); }),
                        skeletons.end());
        return true;
      }

      std::ostream& operator<<(std::ostream& t_os, const SkeletonGroup& t_sg)
      {
        return t_os << utils::toString(t_sg);
      }

    } // namespace types
  } // namespace skeletons
} // namespace open_ptrack
