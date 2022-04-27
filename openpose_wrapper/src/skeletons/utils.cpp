// Eigen dependencies
#include "eigen3/Eigen/Geometry"
#include "eigen3/Eigen/SVD"

// Internal dependencies
#include "openpose_wrapper/skeletons/utils.h"

const std::string PAD = "  ";

const std::string open_ptrack::skeletons::utils::padding(int t_n_pads)
{
  std::string ret_str;
  while (--t_n_pads >= 0) {
    ret_str += PAD;
  }
  return ret_str;
}

// Vector3
open_ptrack::skeletons::types::Vector3
open_ptrack::skeletons::utils::toStruct(const double& t_x, const double& t_y, const double& t_z)
{
  return open_ptrack::skeletons::types::Vector3(t_x, t_y, t_z);
}

open_ptrack::skeletons::types::Vector3
open_ptrack::skeletons::utils::toStruct(const geometry_msgs::Vector3& t_v)
{
  return open_ptrack::skeletons::types::Vector3(t_v.x, t_v.y, t_v.z);
}

geometry_msgs::Vector3
open_ptrack::skeletons::utils::toVector3Msg(const open_ptrack::skeletons::types::Vector3& t_v)
{
  geometry_msgs::Vector3 v;
  v.x = t_v.x();
  v.y = t_v.y();
  v.z = t_v.z();
  return v;
}

bool open_ptrack::skeletons::utils::isNaN(const open_ptrack::skeletons::types::Vector3& t_v)
{
  return (std::isnan(t_v.x()) || std::isnan(t_v.y()) || std::isnan(t_v.z()));
}

double open_ptrack::skeletons::utils::magnitude(const open_ptrack::skeletons::types::Vector3& t_v)
{
  return t_v.length();
}

std::string open_ptrack::skeletons::utils::toString(const open_ptrack::skeletons::types::Vector3& t_v,
                                              int t_pad_lv)
{
  std::stringstream ss;
  ss << padding(t_pad_lv) << "- x: " << t_v.x() << std::endl
     << padding(t_pad_lv) << "  y: " << t_v.y() << std::endl
     << padding(t_pad_lv) << "  z: " << t_v.z();
  return ss.str();
}

// Point
open_ptrack::skeletons::types::Point open_ptrack::skeletons::utils::toStruct(const geometry_msgs::Point& t_p)
{
  return open_ptrack::skeletons::types::Point(t_p.x, t_p.y, t_p.z);
}

geometry_msgs::Point open_ptrack::skeletons::utils::toPointMsg(const open_ptrack::skeletons::types::Point& t_p)
{
  geometry_msgs::Point p;
  p.x = t_p.x();
  p.y = t_p.y();
  p.z = t_p.z();
  return p;
}

double open_ptrack::skeletons::utils::distance(const open_ptrack::skeletons::types::Point& t_p1,
                                         const open_ptrack::skeletons::types::Point& t_p2)
{
  return (!std::isnan(t_p1.z()) && !std::isnan(t_p2.z()))
           ? t_p1.distance(t_p2)
           : open_ptrack::skeletons::types::Point(t_p1.x(), t_p1.y(), 0)
               .distance(types::Point(t_p2.x(), t_p2.y(), 0));
}

// Quaternion
open_ptrack::skeletons::types::Quaternion open_ptrack::skeletons::utils::toStruct(const double& t_x,
                                                                      const double& t_y,
                                                                      const double& t_z,
                                                                      const double& t_w)
{
  return open_ptrack::skeletons::types::Quaternion(t_x, t_y, t_z, t_w);
}

open_ptrack::skeletons::types::Quaternion
open_ptrack::skeletons::utils::toStruct(const geometry_msgs::Quaternion& t_q)
{
  return open_ptrack::skeletons::types::Quaternion(t_q.x, t_q.y, t_q.z, t_q.w);
}

geometry_msgs::Quaternion
open_ptrack::skeletons::utils::toMsg(const open_ptrack::skeletons::types::Quaternion& t_q)
{
  geometry_msgs::Quaternion q;
  q.x = t_q.x();
  q.y = t_q.y();
  q.z = t_q.z();
  q.w = t_q.w();
  return q;
}

bool open_ptrack::skeletons::utils::isNaN(const open_ptrack::skeletons::types::Quaternion& t_q)
{
  return (std::isnan(t_q.x()) || std::isnan(t_q.y()) || std::isnan(t_q.z()) || std::isnan(t_q.w()));
}

double open_ptrack::skeletons::utils::distance(const open_ptrack::skeletons::types::Quaternion& t_q1,
                                         const open_ptrack::skeletons::types::Quaternion& t_q2)
{
  return t_q1.normalized().angleShortestPath(t_q2.normalized());
}

std::string open_ptrack::skeletons::utils::toString(const open_ptrack::skeletons::types::Quaternion& t_q,
                                              int t_pad_lv)
{
  std::stringstream ss;
  ss << padding(t_pad_lv) << "- x: " << t_q.x() << std::endl
     << padding(t_pad_lv) << "  y: " << t_q.y() << std::endl
     << padding(t_pad_lv) << "  z: " << t_q.z() << std::endl
     << padding(t_pad_lv) << "  w: " << t_q.w();
  return ss.str();
}

// Pose
open_ptrack::skeletons::types::Pose
open_ptrack::skeletons::utils::toStruct(const open_ptrack::skeletons::types::Point& t_position,
                                  const open_ptrack::skeletons::types::Quaternion& t_orientation)
{
  return open_ptrack::skeletons::types::Pose(t_position, t_orientation);
}

open_ptrack::skeletons::types::Pose
open_ptrack::skeletons::utils::toStruct(const open_ptrack::skeletons::types::Quaternion& t_orientation)
{
  return open_ptrack::skeletons::types::Pose(
    open_ptrack::skeletons::types::Point(std::numeric_limits<double>::quiet_NaN(),
                                   std::numeric_limits<double>::quiet_NaN(),
                                   std::numeric_limits<double>::quiet_NaN()),
    t_orientation);
}

open_ptrack::skeletons::types::Pose open_ptrack::skeletons::utils::toStruct(const geometry_msgs::Pose& t_p)
{
  return open_ptrack::skeletons::types::Pose(toStruct(t_p.position), toStruct(t_p.orientation));
}

geometry_msgs::Pose open_ptrack::skeletons::utils::toMsg(const open_ptrack::skeletons::types::Pose& t_p)
{
  geometry_msgs::Pose p;
  p.position = toPointMsg(t_p.position);
  p.orientation = toMsg(t_p.orientation);
  return p;
}

bool open_ptrack::skeletons::utils::isNaN(const open_ptrack::skeletons::types::Pose& t_p)
{
  return (isNaN(t_p.position) && isNaN(t_p.orientation));
}

std::string open_ptrack::skeletons::utils::toString(const open_ptrack::skeletons::types::Pose& t_p,
                                              int t_pad_lv)
{
  bool print_position = !isNaN(t_p.position) || isNaN(t_p.orientation);
  bool print_orientation = !isNaN(t_p.orientation) || isNaN(t_p.position);

  std::stringstream ss;
  ss << padding(t_pad_lv) << "- ";
  if (print_position) {
    ss << "position:" << std::endl << toString(t_p.position, t_pad_lv + 1);
    if (print_orientation) {
      ss << std::endl << padding(t_pad_lv) << "  ";
    }
  }
  if (print_orientation) {
    ss << "orientation:" << std::endl << toString(t_p.orientation, t_pad_lv + 1);
  }
  return ss.str();
}

// Velocity
open_ptrack::skeletons::types::Velocity
open_ptrack::skeletons::utils::toStruct(const open_ptrack::skeletons::types::Vector3& t_linear,
                                  const open_ptrack::skeletons::types::Vector3& t_angular)
{
  return open_ptrack::skeletons::types::Velocity(t_linear, t_angular);
}

open_ptrack::skeletons::types::Velocity open_ptrack::skeletons::utils::toStruct(const geometry_msgs::Twist& t_v)
{
  return open_ptrack::skeletons::types::Velocity(toStruct(t_v.linear), toStruct(t_v.angular));
}

geometry_msgs::Twist
open_ptrack::skeletons::utils::toTwistMsg(const open_ptrack::skeletons::types::Velocity& t_v)
{
  geometry_msgs::Twist t;
  t.linear = toVector3Msg(t_v.linear);
  t.angular = toVector3Msg(t_v.angular);
  return t;
}

bool open_ptrack::skeletons::utils::isNaN(const open_ptrack::skeletons::types::Velocity& t_v)
{
  return (isNaN(t_v.linear) && isNaN(t_v.angular));
}

std::string open_ptrack::skeletons::utils::toString(const open_ptrack::skeletons::types::Velocity& t_v,
                                              int t_pad_lv)
{
  bool print_linear = !isNaN(t_v.linear) || isNaN(t_v.angular);
  bool print_angular = !isNaN(t_v.angular) || isNaN(t_v.linear);

  std::stringstream ss;
  ss << padding(t_pad_lv) << "- ";
  if (print_linear) {
    ss << "linear:" << std::endl << toString(t_v.linear, t_pad_lv + 1);
    if (print_angular) {
      ss << std::endl << padding(t_pad_lv) << "  ";
    }
  }
  if (print_angular) {
    ss << "angular:" << std::endl << toString(t_v.angular, t_pad_lv + 1);
  }
  return ss.str();
}

// Acceleration
open_ptrack::skeletons::types::Acceleration
open_ptrack::skeletons::utils::toStruct(const geometry_msgs::Accel& t_a)
{
  return open_ptrack::skeletons::types::Acceleration(toStruct(t_a.linear), toStruct(t_a.angular));
}

geometry_msgs::Accel
open_ptrack::skeletons::utils::toAccelMsg(const open_ptrack::skeletons::types::Acceleration& t_a)
{
  geometry_msgs::Accel a;
  a.linear = toVector3Msg(t_a.linear);
  a.angular = toVector3Msg(t_a.angular);
  return a;
}

// KinematicState
open_ptrack::skeletons::types::KinematicState
open_ptrack::skeletons::utils::toStruct(const open_ptrack::skeletons::types::Pose& t_pose,
                                  const open_ptrack::skeletons::types::Velocity& t_velocity,
                                  const open_ptrack::skeletons::types::Acceleration& t_acceleration)
{
  return open_ptrack::skeletons::types::KinematicState(t_pose, t_velocity, t_acceleration);
}

open_ptrack::skeletons::types::KinematicState
open_ptrack::skeletons::utils::toStruct(const openpose_wrapper::KinematicState& t_ks)
{
  return open_ptrack::skeletons::types::KinematicState(
    toStruct(t_ks.pose), toStruct(t_ks.velocity), toStruct(t_ks.acceleration));
}

openpose_wrapper::KinematicState
open_ptrack::skeletons::utils::toMsg(const open_ptrack::skeletons::types::KinematicState& t_ks)
{
  openpose_wrapper::KinematicState ks;
  ks.pose = toMsg(t_ks.pose);
  ks.velocity = toTwistMsg(t_ks.velocity);
  ks.acceleration = toAccelMsg(t_ks.acceleration);
  return ks;
}

std::string open_ptrack::skeletons::utils::toString(const open_ptrack::skeletons::types::KinematicState& t_ks,
                                              int t_pad_lv)
{
  std::stringstream ss;
  ss << padding(t_pad_lv) << "- pose:" << std::endl << toString(t_ks.pose, t_pad_lv + 1);
  if (!isNaN(t_ks.velocity)) {
    ss << std::endl
       << padding(t_pad_lv) << "  velocity:" << std::endl
       << toString(t_ks.velocity, t_pad_lv + 1);
  }
  if (!isNaN(t_ks.acceleration)) {
    ss << std::endl
       << padding(t_pad_lv) << "  acceleration:" << std::endl
       << toString(t_ks.acceleration, t_pad_lv + 1);
  }
  return ss.str();
}

// Box
open_ptrack::skeletons::types::Box
open_ptrack::skeletons::utils::toStruct(const open_ptrack::skeletons::types::KinematicState& t_center,
                                  const double& t_height,
                                  const double& t_length,
                                  const double& t_width)
{
  return open_ptrack::skeletons::types::Box(t_center, t_height, t_length, t_width);
}

open_ptrack::skeletons::types::Box open_ptrack::skeletons::utils::toStruct(const openpose_wrapper::Box& t_b)
{
  return open_ptrack::skeletons::types::Box(toStruct(t_b.center), t_b.height, t_b.length, t_b.width);
}

openpose_wrapper::Box open_ptrack::skeletons::utils::toMsg(const open_ptrack::skeletons::types::Box& t_b)
{
  openpose_wrapper::Box b;
  b.center = toMsg(t_b.center);
  b.height = t_b.height;
  b.length = t_b.length;
  b.width = t_b.width;
  return b;
}

std::string open_ptrack::skeletons::utils::toString(const open_ptrack::skeletons::types::Box& t_b, int t_pad_lv)
{
  std::stringstream ss;
  ss << padding(t_pad_lv) << "- center:" << std::endl
     << toString(t_b.center, t_pad_lv + 1) << std::endl
     << padding(t_pad_lv) << "  height: " << t_b.height << std::endl
     << padding(t_pad_lv) << "  length: " << t_b.length << std::endl
     << padding(t_pad_lv) << "  width: " << t_b.width;
  return ss.str();
}

// Marker
open_ptrack::skeletons::types::Marker
open_ptrack::skeletons::utils::toStruct(const int& t_id,
                                  const std::string& t_name,
                                  const double& t_confidence,
                                  const open_ptrack::skeletons::types::KinematicState& t_center)
{
  return open_ptrack::skeletons::types::Marker(t_id, t_name, t_confidence, t_center);
}

open_ptrack::skeletons::types::Marker
open_ptrack::skeletons::utils::toStruct(const openpose_wrapper::Marker& t_m)
{
  return open_ptrack::skeletons::types::Marker(t_m.id, t_m.name, t_m.confidence, toStruct(t_m.center));
}

openpose_wrapper::Marker
open_ptrack::skeletons::utils::toMsg(const open_ptrack::skeletons::types::Marker& t_m)
{
  openpose_wrapper::Marker m;
  m.id = t_m.id;
  m.name = t_m.name;
  m.confidence = t_m.confidence;
  m.center = toMsg(t_m.center);
  return m;
}

std::string open_ptrack::skeletons::utils::toString(const open_ptrack::skeletons::types::Marker& t_m,
                                              int t_pad_lv)
{
  std::stringstream ss;
  ss << padding(t_pad_lv) << "- id: " << t_m.id << std::endl;
  if (!t_m.name.empty()) {
    ss << padding(t_pad_lv) << "  name: " << t_m.name << std::endl;
  }
  ss << padding(t_pad_lv) << "  confidence: " << t_m.confidence << std::endl
     << padding(t_pad_lv) << "  center:" << std::endl
     << toString(t_m.center, t_pad_lv + 1);
  return ss.str();
}

// Link
open_ptrack::skeletons::types::Link
open_ptrack::skeletons::utils::toStruct(const int& t_id,
                                  const std::string& t_name,
                                  const int& t_parent_marker,
                                  const int& t_child_marker,
                                  const double& t_confidence,
                                  const open_ptrack::skeletons::types::KinematicState& t_center)
{
  return open_ptrack::skeletons::types::Link(
    t_id, t_name, t_parent_marker, t_child_marker, t_confidence, t_center);
}

open_ptrack::skeletons::types::Link
open_ptrack::skeletons::utils::toStruct(const openpose_wrapper::Link& t_l)
{
  return open_ptrack::skeletons::types::Link(
    t_l.id, t_l.name, t_l.parent_marker, t_l.child_marker, t_l.confidence, toStruct(t_l.center));
}

openpose_wrapper::Link open_ptrack::skeletons::utils::toMsg(const open_ptrack::skeletons::types::Link& t_l)
{
  openpose_wrapper::Link l;
  l.id = t_l.id;
  l.name = t_l.name;
  l.parent_marker = t_l.parent_marker;
  l.child_marker = t_l.child_marker;
  l.confidence = t_l.confidence;
  l.center = toMsg(t_l.center);
  return l;
}

std::string open_ptrack::skeletons::utils::toString(const open_ptrack::skeletons::types::Link& t_l,
                                              int t_pad_lv)
{
  std::stringstream ss;
  ss << padding(t_pad_lv) << "- id: " << t_l.id << std::endl;
  if (!t_l.name.empty()) {
    ss << padding(t_pad_lv) << "  name: " << t_l.name << std::endl;
  }
  ss << padding(t_pad_lv) << "  parent_marker: " << t_l.parent_marker << std::endl
     << padding(t_pad_lv) << "  child_marker: " << t_l.child_marker << std::endl
     << padding(t_pad_lv) << "  confidence: " << t_l.confidence << std::endl
     << padding(t_pad_lv) << "  center:" << std::endl
     << toString(t_l.center, t_pad_lv + 1);
  return ss.str();
}

// Skeleton
open_ptrack::skeletons::types::Skeleton
open_ptrack::skeletons::utils::toStruct(const int& t_id,
                                  const double& t_src_time,
                                  const std::string& t_src_frame,
                                  const unsigned int& t_max_markers,
                                  const unsigned int& t_max_links,
                                  const double& t_confidence,
                                  const open_ptrack::skeletons::types::Box& t_bounding_box,
                                  const std::vector<open_ptrack::skeletons::types::Marker>& t_markers,
                                  const std::vector<open_ptrack::skeletons::types::Link>& t_links)
{
  return open_ptrack::skeletons::types::Skeleton(t_id,
                                           t_src_time,
                                           t_src_frame,
                                           t_max_markers,
                                           t_max_links,
                                           t_confidence,
                                           t_bounding_box,
                                           t_markers,
                                           t_links);
}

open_ptrack::skeletons::types::Skeleton
open_ptrack::skeletons::utils::toStruct(const openpose_wrapper::Skeleton& t_s)
{
  open_ptrack::skeletons::types::Skeleton s(t_s.id,
                                      t_s.src_time.toSec(),
                                      t_s.src_frame,
                                      t_s.max_markers,
                                      t_s.max_links,
                                      t_s.confidence,
                                      toStruct(t_s.bounding_box));
  for (auto& m : t_s.markers) {
    s.addMarker(toStruct(m));
  }
  for (auto& l : t_s.links) {
    s.addLink(toStruct(l));
  }
  return s;
}

openpose_wrapper::Skeleton
open_ptrack::skeletons::utils::toMsg(const open_ptrack::skeletons::types::Skeleton& t_s)
{
  openpose_wrapper::Skeleton s;
  s.id = t_s.id;
  s.src_time = ros::Time(t_s.src_time);
  s.src_frame = t_s.src_frame;
  s.max_markers = t_s.max_markers;
  s.max_links = t_s.max_links;
  s.confidence = t_s.confidence;
  s.bounding_box = toMsg(t_s.bounding_box);
  s.markers.reserve(t_s.markers.size());
  s.links.reserve(t_s.links.size());
  for (auto& m : t_s.markers) {
    s.markers.push_back(toMsg(m));
  }
  for (auto& l : t_s.links) {
    s.links.push_back(toMsg(l));
  }
  return s;
}

open_ptrack::skeletons::types::Box
open_ptrack::skeletons::utils::computeBoundingBox(const open_ptrack::skeletons::types::Skeleton& t_s)
{
  if (t_s.markers.empty()) {
    return types::Box();
  }

  auto max_markers = t_s.markers.size();

  Eigen::MatrixXd data(max_markers, 3);
  unsigned int i = 0;
  for (const auto& mk : t_s.markers) {
    data.row(i++) << mk.center.pose.position.x(), mk.center.pose.position.y(),
      mk.center.pose.position.z();
  }
  // Replace NaNs with zeros
  data = (!data.array().isNaN()).select(data, 0);

  Eigen::MatrixXd centered = data.rowwise() - data.colwise().mean();
  Eigen::MatrixXd covariance =
    (centered.adjoint() * centered) / static_cast<double>((max_markers - 1));

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(covariance, Eigen::ComputeThinU);
  Eigen::Matrix3d svd_u = svd.matrixU();
  if (svd_u.determinant() < 0.0) {
    // Trick to force U to be a rotation matrix
    svd_u.col(0) = -svd_u.col(0);
  }

  Eigen::Quaterniond orientation_q(svd_u);

  Eigen::MatrixXd svd_data = data * svd_u;
  auto svd_min_point = svd_data.colwise().minCoeff();
  auto svd_max_point = svd_data.colwise().maxCoeff();
  auto svd_center_point = (svd_min_point + svd_max_point) / 2.0;

  auto center_point = svd_u * svd_center_point.transpose();

  return open_ptrack::skeletons::types::Box(
    {{{center_point(0), center_point(1), center_point(2)},
      {orientation_q.x(), orientation_q.y(), orientation_q.z(), orientation_q.w()}}},
    svd_max_point(0) - svd_min_point(0),
    svd_max_point(1) - svd_min_point(1),
    svd_max_point(2) - svd_min_point(2));
}

open_ptrack::skeletons::types::KinematicState
open_ptrack::skeletons::utils::centroid(const open_ptrack::skeletons::types::Skeleton& t_s)
{
  return isNaN(t_s.bounding_box.center.pose) ? computeBoundingBox(t_s).center
                                             : t_s.bounding_box.center;
}

std::string open_ptrack::skeletons::utils::toString(const open_ptrack::skeletons::types::Skeleton& t_s,
                                              int t_pad_lv)
{
  std::stringstream ss;
  ss << padding(t_pad_lv) << "- id: " << t_s.id << std::endl << padding(t_pad_lv) << "  src_time: ";
  if (!std::isnan(t_s.src_time)) {
    long src_time_sec = static_cast<long>(t_s.src_time);
    long src_time_nsec = static_cast<long>((t_s.src_time - src_time_sec) * 1e9);
    ss << std::to_string(src_time_sec) << "." << std::to_string(src_time_nsec);
  }
  else {
    ss << "nan";
  }
  ss << std::endl
     << padding(t_pad_lv) << "  src_frame: " << t_s.src_frame << std::endl
     << padding(t_pad_lv) << "  max_markers: " << t_s.max_markers << std::endl
     << padding(t_pad_lv) << "  max_links: " << t_s.max_links << std::endl
     << padding(t_pad_lv) << "  confidence: " << t_s.confidence << std::endl
     << padding(t_pad_lv) << "  bounding_box:";
  if (isNaN(t_s.bounding_box.center.pose)) {
    ss << " []";
  }
  else {
    ss << std::endl << toString(t_s.bounding_box, t_pad_lv + 1);
  }
  ss << std::endl << padding(t_pad_lv) << "  markers:";
  if (t_s.markers.empty()) {
    ss << " []";
  }
  else {
    for (auto m : t_s.markers) {
      ss << std::endl << toString(m, t_pad_lv + 1);
    }
  }
  ss << std::endl << padding(t_pad_lv) << "  links:";
  if (t_s.links.empty()) {
    ss << " []";
  }
  else {
    for (auto l : t_s.links) {
      ss << std::endl << toString(l, t_pad_lv + 1);
    }
  }
  return ss.str();
}

// SkeletonGroup
open_ptrack::skeletons::types::SkeletonGroup
open_ptrack::skeletons::utils::toStruct(const double& t_time,
                                  const std::string& t_frame,
                                  const std::vector<open_ptrack::skeletons::types::Skeleton>& t_skeletons)
{
  return open_ptrack::skeletons::types::SkeletonGroup(t_time, t_frame, t_skeletons);
}

open_ptrack::skeletons::types::SkeletonGroup
open_ptrack::skeletons::utils::toStruct(const openpose_wrapper::SkeletonGroup& t_sg)
{
  open_ptrack::skeletons::types::SkeletonGroup sg;
  sg.time = t_sg.header.stamp.toSec();
  sg.frame = t_sg.header.frame_id;
  sg.skeletons.reserve(t_sg.skeletons.size());
  for (auto& s : t_sg.skeletons) {
    sg.skeletons.push_back(toStruct(s));
  }
  return sg;
}

openpose_wrapper::SkeletonGroup
open_ptrack::skeletons::utils::toMsg(const open_ptrack::skeletons::types::SkeletonGroup& t_sg)
{
  openpose_wrapper::SkeletonGroup sg;
  sg.header.stamp = ros::Time(t_sg.time);
  sg.header.frame_id = t_sg.frame;
  sg.skeletons.reserve(t_sg.skeletons.size());
  for (auto& s : t_sg.skeletons) {
    sg.skeletons.push_back(toMsg(s));
  }
  return sg;
}

std::string open_ptrack::skeletons::utils::toString(const open_ptrack::skeletons::types::SkeletonGroup& t_sg,
                                              int t_pad_lv)
{
  std::stringstream ss;
  ss << padding(t_pad_lv) << "- time: ";
  if (!std::isnan(t_sg.time)) {
    long src_time_sec = static_cast<long>(t_sg.time);
    long src_time_nsec = static_cast<long>((t_sg.time - src_time_sec) * 1e9);
    ss << std::to_string(src_time_sec) << "." << std::to_string(src_time_nsec);
  }
  else {
    ss << "nan";
  }
  ss << std::endl
     << padding(t_pad_lv) << "  frame: " << t_sg.frame << std::endl
     << padding(t_pad_lv) << "  skeletons:";
  if (t_sg.skeletons.empty()) {
    ss << " []";
  }
  else {
    for (auto s : t_sg.skeletons) {
      ss << std::endl << toString(s, t_pad_lv + 1);
    }
  }
  return ss.str();
}
