#include "openpose_wrapper/projector.h"

#include <limits>
#include <numeric>

#include "cv_bridge/cv_bridge.h"

#include "openpose_wrapper/skeletons/utils.h"

open_ptrack::opw3d::Projector::Projector()
  : m_nh("~")
  , m_node_namespace(m_nh.getNamespace())
  , m_depth_queue(new open_ptrack::opw3d::Queue){};

void open_ptrack::opw3d::Projector::configure()
{
  ROS_INFO_STREAM("Hi-ROS 3D Projector...Configuring");

  if (m_configured) {
    m_configured = false;
    stop();
  }

  m_nh.getParam("node_name", m_params.node_name);

  m_nh.getParam("in_skeleton_group_topic", m_params.in_skeleton_group_topic);
  m_nh.getParam("in_depth_topic", m_params.in_depth_topic);
  m_nh.getParam("in_camera_info_topic", m_params.in_camera_info_topic);

  m_nh.getParam("out_msg_topic_name", m_params.out_msg_topic_name);

  m_nh.getParam("publish_in_global_reference_frame", m_params.publish_in_global_reference_frame);
  m_nh.getParam("global_reference_frame", m_params.global_reference_frame);

  m_nh.getParam("depth_to_meters_multiplier", m_params.depth_to_meters_multiplier);

  m_nh.getParam("enable_bounding_box", m_params.enable_bounding_box);
  m_nh.getParam("epsilon_time_matching", m_params.epsilon_time_matching);

  m_nh.getParam("bb_equivalent_side_scaling_factor", m_private_params.bb_equivalent_side_scaling_factor);
  m_nh.getParam("valid_to_total_mks_scaling_factor", m_private_params.valid_to_total_mks_scaling_factor);

  if (m_params.publish_in_global_reference_frame) {
    ros::Duration(2).sleep();
    if (!m_tf_listener.frameExists(m_params.global_reference_frame)) {
      ROS_FATAL_STREAM("Global reference frame <" << m_params.global_reference_frame
                                                  << "> does not exist. Unable to continue");
      ros::shutdown();
    }
    m_private_params.tf_configured = false;
  }

  m_configured = true;
  ROS_INFO_STREAM(BASH_MSG_GREEN << "Hi-ROS 3D Projector...CONFIGURED" << BASH_MSG_RESET);
}

void open_ptrack::opw3d::Projector::start()
{
  ROS_INFO_STREAM("Hi-ROS 3D Projector...Starting");

  if (!m_configured) {
    configure();
  }

  setupRosTopics();

  ROS_INFO_STREAM(BASH_MSG_GREEN << "Hi-ROS 3D Projector...RUNNING" << BASH_MSG_RESET);
  ros::spin();
}

void open_ptrack::opw3d::Projector::stop()
{
  ROS_INFO_STREAM("Hi-ROS 3D Projector...Stopping");

  if (m_in_depth_sub) {
    m_in_depth_sub.shutdown();
  }

  if (m_in_camera_info_sub) {
    m_in_camera_info_sub.shutdown();
  }

  if (m_in_skeleton_group_sub) {
    m_in_skeleton_group_sub.shutdown();
  }

  if (m_out_msg_pub) {
    m_out_msg_pub.shutdown();
  }

  ROS_INFO_STREAM(BASH_MSG_GREEN << "Hi-ROS 3D Projector...STOPPED" << BASH_MSG_RESET);
}

void open_ptrack::opw3d::Projector::setupRosTopics()
{
  m_in_depth_sub = m_nh.subscribe(m_params.in_depth_topic, 1, &Projector::pushDepth, this);

  m_in_camera_info_sub = m_nh.subscribe(m_params.in_camera_info_topic, 1, &Projector::setCameraInfo, this);
  while (m_in_depth_sub.getNumPublishers() == 0 && !ros::isShuttingDown()) {
    ROS_WARN_STREAM_THROTTLE(2, m_node_namespace << " No input messages on depth topic");
  }

  m_in_skeleton_group_sub = m_nh.subscribe(m_params.in_skeleton_group_topic, 1, &Projector::to3dCallback, this);
  while (m_in_skeleton_group_sub.getNumPublishers() == 0 && !ros::isShuttingDown()) {
    ROS_WARN_STREAM_THROTTLE(2, m_node_namespace << " No input messages on skeleton group topic");
  }

  m_out_msg_pub = m_nh.advertise<openpose_wrapper::SkeletonGroup>(m_params.out_msg_topic_name, 1);
}

void open_ptrack::opw3d::Projector::configureGlobalFrameConverter()
{
  if (!m_tf_listener.frameExists(m_private_params.local_reference_frame)) {
    ROS_FATAL_STREAM("Cannot find tf between <" << m_private_params.local_reference_frame << "> and <"
                                                << m_params.global_reference_frame << ">. Unable to continue");
    ros::shutdown();
  }
}

std::string open_ptrack::opw3d::Projector::extractImageQualityFromTopicString(const std::string& t_topic_name) const
{
  auto tmp = t_topic_name.find_last_of("/");
  std::string tmp2 = t_topic_name.substr(0, tmp - 1);
  auto tmp3 = tmp2.find_last_of("/");
  return t_topic_name.substr(tmp3 + 1, tmp - tmp3 - 1);
}

void open_ptrack::opw3d::Projector::pushDepth(sensor_msgs::ImageConstPtr t_depth)
{
  if (!m_private_params.camera_info_configured) {
    return;
  }

  if (!m_private_params.depth_configured) {
    if (t_depth.get()->width != m_private_params.width || t_depth.get()->height != m_private_params.height) {
      ROS_FATAL_STREAM("Hi-ROS 3D Projector...RGB and depth size mismatch. Unable to continue");
      ros::shutdown();
    }
    m_private_params.depth_configured = true;
  }

  m_depth_queue.get()->push_back(t_depth);
}

void open_ptrack::opw3d::Projector::setCameraInfo(sensor_msgs::CameraInfoConstPtr t_camera_info)
{
  if (!m_private_params.camera_info_configured) {
    m_private_params.width = t_camera_info.get()->width;
    m_private_params.height = t_camera_info.get()->height;

    auto& k_matrix = t_camera_info.get()->K;

    if (std::all_of(k_matrix.begin(), k_matrix.end(), [](double i) { return i == 0.0; })) {
      ROS_FATAL_STREAM("Hi-ROS 3D Projector...Source camera is not calibrated");
      ros::shutdown();
    }

    for (unsigned int k = 0; k < k_matrix.size(); ++k) {
      m_private_params.camera_info_k_matrix.at(k) = k_matrix.at(k);
    }
    m_private_params.camera_info_configured = true;
  }
}

void open_ptrack::opw3d::Projector::to3dCallback(openpose_wrapper::SkeletonGroupConstPtr t_skeleton_group_msg)
{
  if (!m_private_params.camera_info_configured) {
    ROS_WARN_STREAM("Camera Calibration not available. Skipping this frame");
    return;
  }

  if (t_skeleton_group_msg->skeletons.empty()) {
    return;
  }

  if (!m_private_params.tf_configured) {
    m_private_params.local_reference_frame = t_skeleton_group_msg->skeletons.front().src_frame;

    if (m_params.publish_in_global_reference_frame) {
      configureGlobalFrameConverter();
      m_private_params.output_reference_frame = m_params.global_reference_frame;
    }
    else {
      m_private_params.output_reference_frame = m_private_params.local_reference_frame;
    }

    m_private_params.tf_configured = true;
  }

  ros::Time skeleton_group_src_time = t_skeleton_group_msg->skeletons.front().src_time;
  sensor_msgs::ImageConstPtr depth_img = m_depth_queue->get(skeleton_group_src_time);
  if (!depth_img) {
    ROS_WARN_STREAM("No depth image available to match skeleton group src time. Skipping this frame");
    return;
  }
  std::shared_ptr<cv::Mat> depth_mat = std::make_shared<cv::Mat>(cv_bridge::toCvShare(depth_img)->image);

  skeletons::types::SkeletonGroup skeleton_group_3d = project(t_skeleton_group_msg, *depth_mat);

  // publish message with 3d skeleton group
  m_out_msg_pub.publish(skeletons::utils::toMsg(skeleton_group_3d));

  m_depth_queue->clear_until_time(skeleton_group_src_time);
}

open_ptrack::skeletons::types::SkeletonGroup
open_ptrack::opw3d::Projector::project(openpose_wrapper::SkeletonGroupConstPtr t_skeleton_group,
                                 const cv::Mat& t_depth_mat)
{
  skeletons::types::SkeletonGroup skeleton_group_3d = skeletons::utils::toStruct(*t_skeleton_group);
  skeleton_group_3d.time = ros::Time::now().toSec();
  skeleton_group_3d.frame = m_private_params.output_reference_frame;

  for (auto& sk : skeleton_group_3d.skeletons) {
    cv::Rect depth_averaging_rect = computeDepthAveragingArea(sk);

    // Markers
    for (int i = static_cast<int>(sk.markers.size()) - 1; i >= 0; --i) {
      auto& marker = sk.markers.at(static_cast<unsigned int>(i));

      depth_averaging_rect.x = inRange(
        0, static_cast<int>(marker.center.pose.position.x() - depth_averaging_rect.width * 0.5), t_depth_mat.cols - 1);
      depth_averaging_rect.y = inRange(
        0, static_cast<int>(marker.center.pose.position.y() - depth_averaging_rect.height * 0.5), t_depth_mat.rows - 1);
      depth_averaging_rect.width = inRange(1, depth_averaging_rect.width, t_depth_mat.cols - depth_averaging_rect.x);
      depth_averaging_rect.height = inRange(1, depth_averaging_rect.height, t_depth_mat.rows - depth_averaging_rect.y);

      marker.center.pose.position.setZ(toMeters(getDepthMedianValue(t_depth_mat(depth_averaging_rect))));

      if (std::isnan(marker.center.pose.position.z())) {
        sk.removeMarker(marker.id);
      }
      else {
        marker.center.pose.position.setX(
          ((marker.center.pose.position.x() - (m_private_params.cx)) * marker.center.pose.position.z())
          / (m_private_params.fx));
        marker.center.pose.position.setY(
          ((marker.center.pose.position.y() - (m_private_params.cy)) * marker.center.pose.position.z())
          / (m_private_params.fy));

        if (m_params.publish_in_global_reference_frame && m_private_params.tf_configured) {
          toGlobalReferenceFrame(marker.center);
        }
      }
    }

    // Links
    for (auto& link : sk.links) {
      if (sk.hasMarker(link.parent_marker) && sk.hasMarker(link.child_marker)) {
        link.center.pose.position =
          sk.getMarker(link.parent_marker)
            .center.pose.position.lerp(sk.getMarker(link.child_marker).center.pose.position, 0.5);
      }
      else {
        link.center.pose.position = skeletons::types::Point(std::numeric_limits<double>::quiet_NaN(),
                                                            std::numeric_limits<double>::quiet_NaN(),
                                                            std::numeric_limits<double>::quiet_NaN());
      }
    }

    // Remove 2D bounding box data from 3D skeleton group
    sk.bounding_box = skeletons::types::Box();

    if (m_params.enable_bounding_box) {
      sk.bounding_box = skeletons::utils::computeBoundingBox(sk);
    }
  }

  return skeleton_group_3d;
}

double open_ptrack::opw3d::Projector::toMeters(const double& t_x) const
{
  return t_x * m_params.depth_to_meters_multiplier;
}

std::vector<double> open_ptrack::opw3d::Projector::toStdVector(const cv::Mat& t_mat) const
{
  std::vector<double> v;

  switch (t_mat.type()) {
    case CV_8U:
      if (t_mat.isContinuous()) {
        v.assign(reinterpret_cast<uchar*>(t_mat.data),
                 reinterpret_cast<uchar*>(t_mat.data) + t_mat.total() * static_cast<unsigned int>(t_mat.channels()));
      }
      else {
        for (int i = 0; i < t_mat.rows; ++i) {
          v.insert(v.end(), t_mat.ptr<uchar>(i), t_mat.ptr<uchar>(i) + t_mat.cols * t_mat.channels());
        }
      }
      break;

    case CV_8S:
      if (t_mat.isContinuous()) {
        v.assign(reinterpret_cast<char*>(t_mat.data),
                 reinterpret_cast<char*>(t_mat.data) + t_mat.total() * static_cast<unsigned int>(t_mat.channels()));
      }
      else {
        for (int i = 0; i < t_mat.rows; ++i) {
          v.insert(v.end(), t_mat.ptr<char>(i), t_mat.ptr<char>(i) + t_mat.cols * t_mat.channels());
        }
      }
      break;

    case CV_16U:
      if (t_mat.isContinuous()) {
        v.assign(reinterpret_cast<ushort*>(t_mat.data),
                 reinterpret_cast<ushort*>(t_mat.data) + t_mat.total() * static_cast<unsigned int>(t_mat.channels()));
      }
      else {
        for (int i = 0; i < t_mat.rows; ++i) {
          v.insert(v.end(), t_mat.ptr<ushort>(i), t_mat.ptr<ushort>(i) + t_mat.cols * t_mat.channels());
        }
      }
      break;

    case CV_16S:
      if (t_mat.isContinuous()) {
        v.assign(reinterpret_cast<short*>(t_mat.data),
                 reinterpret_cast<short*>(t_mat.data) + t_mat.total() * static_cast<unsigned int>(t_mat.channels()));
      }
      else {
        for (int i = 0; i < t_mat.rows; ++i) {
          v.insert(v.end(), t_mat.ptr<short>(i), t_mat.ptr<short>(i) + t_mat.cols * t_mat.channels());
        }
      }
      break;

    case CV_32S:
      if (t_mat.isContinuous()) {
        v.assign(reinterpret_cast<int*>(t_mat.data),
                 reinterpret_cast<int*>(t_mat.data) + t_mat.total() * static_cast<unsigned int>(t_mat.channels()));
      }
      else {
        for (int i = 0; i < t_mat.rows; ++i) {
          v.insert(v.end(), t_mat.ptr<int>(i), t_mat.ptr<int>(i) + t_mat.cols * t_mat.channels());
        }
      }
      break;

    case CV_32F:
      if (t_mat.isContinuous()) {
        v.assign(reinterpret_cast<float*>(t_mat.data),
                 reinterpret_cast<float*>(t_mat.data) + t_mat.total() * static_cast<unsigned int>(t_mat.channels()));
      }
      else {
        for (int i = 0; i < t_mat.rows; ++i) {
          v.insert(v.end(), t_mat.ptr<float>(i), t_mat.ptr<float>(i) + t_mat.cols * t_mat.channels());
        }
      }
      break;

    case CV_64F:
      if (t_mat.isContinuous()) {
        v.assign(reinterpret_cast<double*>(t_mat.data),
                 reinterpret_cast<double*>(t_mat.data) + t_mat.total() * static_cast<unsigned int>(t_mat.channels()));
      }
      else {
        for (int i = 0; i < t_mat.rows; ++i) {
          v.insert(v.end(), t_mat.ptr<double>(i), t_mat.ptr<double>(i) + t_mat.cols * t_mat.channels());
        }
      }
      break;
  }

  return v;
}

cv::Rect open_ptrack::opw3d::Projector::computeDepthAveragingArea(const open_ptrack::skeletons::types::Skeleton& t_skeleton)
{
  const unsigned int n_valid_markers = static_cast<unsigned int>(t_skeleton.markers.size());
  const unsigned int t_max_markers = t_skeleton.max_markers;
  const int equivalent_bb_side =
    static_cast<int>(std::sqrt(t_skeleton.bounding_box.height * t_skeleton.bounding_box.length));
  const double valid_mks_scaling_factor =
    t_skeleton.markers.size()
    / std::max(static_cast<double>(n_valid_markers),
               t_max_markers / m_private_params.valid_to_total_mks_scaling_factor);

  const int mk_depth_averaging_box_half_side =
    std::max(1,
             static_cast<int>(0.5 * m_private_params.bb_equivalent_side_scaling_factor * valid_mks_scaling_factor
                              * equivalent_bb_side));
  cv::Rect depth_averaging_rect = {0, 0, 2 * mk_depth_averaging_box_half_side, 2 * mk_depth_averaging_box_half_side};
  return depth_averaging_rect;
}

double open_ptrack::opw3d::Projector::getDepthMeanValue(const cv::Mat& t_roi) const
{
  double mean = cv::mean(t_roi).val[0];

  if (mean == 0.0) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  else {
    return mean;
  }
}

double open_ptrack::opw3d::Projector::getDepthMedianValue(const cv::Mat& t_roi) const
{
  std::vector<double> std_roi = toStdVector(t_roi);
  std::remove(std_roi.begin(), std_roi.end(), 0);

  std::nth_element(std_roi.begin(), std_roi.begin() + std_roi.size() / 2, std_roi.end());
  double median = std_roi.at(std_roi.size() / 2);

  if (median == 0.0) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  else {
    return median;
  }
}

void open_ptrack::opw3d::Projector::toGlobalReferenceFrame(open_ptrack::skeletons::types::KinematicState& t_ks) const
{
  bool nan_position = skeletons::utils::isNaN(t_ks.pose.position);
  bool nan_orientation = skeletons::utils::isNaN(t_ks.pose.orientation);

  // Fill missing positions/orientations for transformPose()
  if (nan_position) {
    t_ks.pose.position = skeletons::types::Point(0, 0, 0);
  }
  if (nan_orientation) {
    t_ks.pose.orientation = skeletons::types::Quaternion(0, 0, 0, 1);
  }

  geometry_msgs::PoseStamped in_ks, out_ks;
  in_ks.header.frame_id = m_private_params.local_reference_frame;
  in_ks.pose = skeletons::utils::toMsg(t_ks.pose);
  m_tf_listener.transformPose(m_params.global_reference_frame, in_ks, out_ks);
  t_ks = skeletons::utils::toStruct(out_ks.pose);

  // Reset missing positions/orientations to NaN
  if (nan_position) {
    t_ks.pose.position = skeletons::types::Point(std::numeric_limits<double>::quiet_NaN(),
                                                 std::numeric_limits<double>::quiet_NaN(),
                                                 std::numeric_limits<double>::quiet_NaN());
  }
  if (nan_orientation) {
    t_ks.pose.orientation = skeletons::types::Quaternion(std::numeric_limits<double>::quiet_NaN(),
                                                         std::numeric_limits<double>::quiet_NaN(),
                                                         std::numeric_limits<double>::quiet_NaN(),
                                                         std::numeric_limits<double>::quiet_NaN());
  }
}
