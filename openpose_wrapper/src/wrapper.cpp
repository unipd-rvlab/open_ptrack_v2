// ROS dependencies
#include <ros/ros.h>

// ROS External Packages dependencies
#include <cv_bridge/cv_bridge.h>

// Ros Distributed Message dependencies
#include <sensor_msgs/image_encodings.h>

// Non-ROS External Dependencies
#include <opencv2/core.hpp>

// OpenPose dependencies
#include <openpose/utilities/flagsToOpenPose.hpp>

// Custom Ros Message dependencies
#include "openpose_wrapper/SkeletonGroup.h"

// Custom External Packages dependencies
#include "openpose_wrapper/skeletons/utils.h"

// Internal dependencies
#include "openpose_wrapper/commons/commons.h"
#include "openpose_wrapper/consumer.h"
#include "openpose_wrapper/producer.h"
#include "openpose_wrapper/wrapper.h"

open_ptrack::opw::Wrapper::Wrapper()
  : m_nh("~")
  , m_node_namespace(m_nh.getNamespace())
  , m_ith(m_nh){};

open_ptrack::opw::Wrapper::~Wrapper(){};

void open_ptrack::opw::Wrapper::start()
{
  ROS_INFO_STREAM("OpenPTrack OpenPose Wrapper...Starting");

  if (!m_opw_configured) {
    configure();
  }

  setupRosTopics();

  m_openpose.start();

  ROS_INFO_STREAM(BASH_MSG_GREEN << "OpenPTrack OpenPose Wrapper...RUNNING" << BASH_MSG_RESET);
  ros::spin();
}

void open_ptrack::opw::Wrapper::stop()
{
  ROS_INFO_STREAM("OpenPTrack OpenPose Wrapper...Stopping");

  if (m_openpose.isRunning()) {
    m_openpose.stop();
  }

  if (m_in_img_sub) {
    m_in_img_sub.shutdown();
  }

  if (m_out_img_pub) {
    m_out_img_pub.shutdown();
  }

  if (m_out_msg_pub) {
    m_out_msg_pub.shutdown();
  }

  ROS_INFO_STREAM(BASH_MSG_GREEN << "OpenPTrack OpenPose Wrapper...STOPPED" << BASH_MSG_RESET);
}

void open_ptrack::opw::Wrapper::setupRosTopics()
{
  // Subscribe to input image topic
  m_in_img_sub = m_ith.subscribe(m_opw_params.in_image_topic, 1, &Wrapper::run, this);

  // Sanity check on input image topic
  while (m_in_img_sub.getNumPublishers() == 0 && !ros::isShuttingDown()) {
    ROS_WARN_STREAM_THROTTLE(2, m_node_namespace << " No input messages");
  }

  if (m_opw_params.publish_rendered_image_topic) {
    m_out_img_pub = m_ith.advertise(m_opw_params.out_image_topic_name, 1);
  }

  if (m_opw_params.publish_skeleton_group_topic) {
    m_out_msg_pub = m_nh.advertise<openpose_wrapper::SkeletonGroup>(m_opw_params.out_msg_topic_name, 1);
  }
}

void open_ptrack::opw::Wrapper::configure()
{
  ROS_INFO_STREAM("OpenPTrack OpenPose Wrapper...Configuring");

  if (m_opw_configured) {
    m_opw_configured = false;
    stop();
  }

  configureWrapper();

  configureOpenPose();

  checkInOutConfiguration();
  m_opw_configured = true;

  ROS_INFO_STREAM(BASH_MSG_GREEN << "OpenPTrack OpenPose Wrapper...CONFIGURED" << BASH_MSG_RESET);
};

void open_ptrack::opw::Wrapper::checkInOutConfiguration()
{
  if (m_opw_params.in_image_topic.empty()) {
    ROS_FATAL_STREAM(m_node_namespace << " Empty input topic. No source to read from");
    ros::shutdown();
  }

  if (m_opw_params.publish_rendered_image_topic && m_op_params.render_mode == 0) {
    ROS_WARN_STREAM(m_node_namespace << " OpenPose rendering is disabled. Disabling output image topic as consequence");
    m_opw_params.publish_rendered_image_topic = false;
  }

  if (!m_opw_params.publish_rendered_image_topic && !m_opw_params.publish_skeleton_group_topic) {
    ROS_FATAL_STREAM(m_node_namespace << " Both rendered images and skeleton group topics disabled. Nothing to do");
    ros::shutdown();
  }

  if (m_op_params.models_path.empty()) {
    ROS_FATAL_STREAM(m_node_namespace << " Openpose model path is empty. Impossible to continue");
    ros::shutdown();
  }
}

void open_ptrack::opw::Wrapper::configureWrapper()
{
  m_nh.getParam("input_image_topic", m_opw_params.in_image_topic);

  m_nh.getParam("publish_rendered_image_topic", m_opw_params.publish_rendered_image_topic);
  m_nh.getParam("publish_skeleton_group_topic", m_opw_params.publish_skeleton_group_topic);
  m_nh.getParam("output_image_topic_name", m_opw_params.out_image_topic_name);
  m_nh.getParam("out_msg_topic_name", m_opw_params.out_msg_topic_name);

  m_nh.getParam("use_region_of_interest", m_opw_params.use_region_of_interest);
  m_nh.getParam("lag_minimization_enable", m_opw_params.lag_minimization_enable);

  m_nh.getParam("min_skeleton_confidence", m_opw_params.min_skeleton_confidence);
  m_nh.getParam("min_marker_confidence", m_opw_params.min_marker_confidence);
  m_nh.getParam("enable_marker_filtering", m_opw_params.enable_marker_filtering);

  if (m_opw_params.enable_marker_filtering) {
    XmlRpc::XmlRpcValue xml_markers_to_discard;
    m_nh.getParam("markers_to_discard", xml_markers_to_discard);

    for (int i = 0; i < xml_markers_to_discard.size(); ++i) {
      m_opw_params.markers_to_discard.push_back(xml_markers_to_discard[i]["id"]);
    }
  }

  XmlRpc::XmlRpcValue xml_links;
  m_nh.getParam("links", xml_links);

  for (int i = 0; i < xml_links.size(); ++i) {
    m_opw_params.links_config.push_back(
      LinkProperties({xml_links[i]["id"], xml_links[i]["parent_marker_id"], xml_links[i]["child_marker_id"]}));
  }

  if (m_opw_params.use_region_of_interest) {
    m_nh.getParam("roi_top_left_corner_x", m_opw_params.roi_top_left_corner_x);
    m_nh.getParam("roi_top_left_corner_y", m_opw_params.roi_top_left_corner_y);
    m_nh.getParam("roi_width", m_opw_params.roi_width);
    m_nh.getParam("roi_height", m_opw_params.roi_height);

    m_region_of_interest = cv::Rect(m_opw_params.roi_top_left_corner_x,
                                    m_opw_params.roi_top_left_corner_y,
                                    m_opw_params.roi_width,
                                    m_opw_params.roi_height);

    if (m_region_of_interest.area() == 0) {
      ROS_WARN_STREAM(m_node_namespace << " Region of Interest enabled but empty. Disabling");
      m_opw_params.use_region_of_interest = false;
    }
  }

  if (m_opw_params.lag_minimization_enable) {
    m_nh.getParam("max_input_delay", m_opw_params.max_input_delay);
  }
}

void open_ptrack::opw::Wrapper::configureOpenPose()
{
  //----------------------------
  // Generic OpenPose parameters
  //----------------------------
  m_nh.getParam("number_of_gpus", m_op_params.number_of_gpus);
  m_nh.getParam("gpu_number_start", m_op_params.gpu_number_start);
  m_nh.getParam("max_fps", m_op_params.max_fps);
  m_nh.getParam("upsampling_ratio", m_op_params.upsampling_ratio);
  m_nh.getParam("models_path", m_op_params.models_path);
  m_nh.getParam("output_size", m_op_params.output_size);
  m_nh.getParam("keypoints_scale_mode", m_op_params.keypoints_scale_mode);
  m_nh.getParam("max_number_of_people", m_op_params.max_number_of_people);
  m_nh.getParam("render_mode", m_op_params.render_mode);

  m_nh.getParam("alpha_keypoint", m_op_params.alpha_keypoint);
  m_nh.getParam("alpha_heat_map", m_op_params.alpha_heat_map);
  m_nh.getParam("blend_original_frame", m_op_params.blend_original_frame);
  m_nh.getParam("add_part_candidates", m_op_params.add_part_candidates);
  m_nh.getParam("maximize_positives", m_op_params.maximize_positives);
  m_nh.getParam("enable_google_logging", m_op_params.enable_google_logging);

  //-------------------------
  // Body detector parameters
  //-------------------------
  m_nh.getParam("body_enable_detection", m_op_params.body_enable_detection);
  if (m_op_params.body_enable_detection) {
    m_nh.getParam("body_model_name", m_op_params.body_model_name);
    m_nh.getParam("body_net_input_size", m_op_params.body_net_input_size);
    m_nh.getParam("body_scales_number", m_op_params.body_scales_number);
    m_nh.getParam("body_scale_gap", m_op_params.body_scale_gap);
    m_nh.getParam("body_default_parts_to_render", m_op_params.body_default_parts_to_render);
    m_nh.getParam("body_heat_maps_add_pafs", m_op_params.body_heat_maps_add_pafs);
    m_nh.getParam("body_heat_maps_add_parts", m_op_params.body_heat_maps_add_parts);
    m_nh.getParam("body_heat_maps_add_background", m_op_params.body_heat_maps_add_background);
    m_nh.getParam("body_heat_maps_scale_mode", m_op_params.body_heat_maps_scale_mode);
    m_nh.getParam("body_render_threshold", m_op_params.body_render_threshold);
  }

  //--------------------------
  // Hands detector parameters
  //--------------------------
  m_nh.getParam("hands_enable_detection", m_op_params.hands_enable_detection);
  if (m_op_params.hands_enable_detection) {
    m_nh.getParam("hands_model_name", m_op_params.hands_model_name);
    m_nh.getParam("body_net_input_size", m_op_params.body_net_input_size);
    m_nh.getParam("hands_locations_detector", m_op_params.hands_locations_detector);
    m_nh.getParam("hands_scales_number", m_op_params.hands_scales_number);
    m_nh.getParam("hands_scale_range", m_op_params.hands_scale_range);
    m_nh.getParam("hands_render_threshold", m_op_params.hands_render_threshold);
  }

  //-------------------------
  // Face detector parameters
  //-------------------------
  m_nh.getParam("face_enable_detection", m_op_params.face_enable_detection);
  if (m_op_params.face_enable_detection) {
    m_nh.getParam("face_model_name", m_op_params.face_model_name);
    m_nh.getParam("face_net_input_size", m_op_params.face_net_input_size);
    m_nh.getParam("face_location_detector", m_op_params.face_location_detector);
    m_nh.getParam("face_render_threshold", m_op_params.face_render_threshold);
  }

  if (!m_op_params.body_enable_detection && !m_op_params.hands_enable_detection && !m_op_params.face_enable_detection) {
    ROS_FATAL_STREAM(m_node_namespace << " No detectors enabled. Check your configuration.");
    ros::shutdown();
  }

  const op::WrapperStructPose op_body_configuration{static_cast<op::PoseMode>(m_op_params.body_enable_detection),
                                                    op::flagsToPoint(op::String(m_op_params.body_net_input_size)),
                                                    op::flagsToPoint(op::String(m_op_params.output_size)),
                                                    op::flagsToScaleMode(m_op_params.keypoints_scale_mode),
                                                    m_op_params.number_of_gpus,
                                                    m_op_params.gpu_number_start,
                                                    m_op_params.body_scales_number,
                                                    static_cast<float>(m_op_params.body_scale_gap),
                                                    op::flagsToRenderMode(m_op_params.render_mode),
                                                    op::flagsToPoseModel(op::String(m_op_params.body_model_name)),
                                                    m_op_params.blend_original_frame,
                                                    static_cast<float>(m_op_params.alpha_keypoint),
                                                    static_cast<float>(m_op_params.alpha_heat_map),
                                                    m_op_params.body_default_parts_to_render,
                                                    op::String(m_op_params.models_path),
                                                    op::flagsToHeatMaps(m_op_params.body_heat_maps_add_parts,
                                                                        m_op_params.body_heat_maps_add_background,
                                                                        m_op_params.body_heat_maps_add_pafs),
                                                    op::flagsToScaleMode(m_op_params.body_heat_maps_scale_mode),
                                                    m_op_params.add_part_candidates,
                                                    static_cast<float>(m_op_params.body_render_threshold),
                                                    m_op_params.max_number_of_people,
                                                    m_op_params.maximize_positives,
                                                    m_op_params.max_fps,
                                                    "",
                                                    "",
                                                    static_cast<float>(m_op_params.upsampling_ratio),
                                                    m_op_params.enable_google_logging};
  m_openpose.configure(op_body_configuration);
  if (m_op_params.hands_enable_detection) {
    const op::WrapperStructHand op_hands_configuration{m_op_params.hands_enable_detection,
                                                       op::flagsToDetector(m_op_params.hands_locations_detector),
                                                       op::flagsToPoint(op::String(m_op_params.hands_net_input_size)),
                                                       m_op_params.hands_scales_number,
                                                       static_cast<float>(m_op_params.hands_scale_range),
                                                       op::flagsToRenderMode(m_op_params.render_mode),
                                                       static_cast<float>(m_op_params.alpha_keypoint),
                                                       static_cast<float>(m_op_params.alpha_heat_map),
                                                       static_cast<float>(m_op_params.hands_render_threshold)};
    m_openpose.configure(op_hands_configuration);
  }

  if (m_op_params.face_enable_detection) {
    const op::WrapperStructFace op_face_configuration{m_op_params.face_enable_detection,
                                                      op::flagsToDetector(m_op_params.face_location_detector),
                                                      op::flagsToPoint(op::String(m_op_params.face_net_input_size)),
                                                      op::flagsToRenderMode(m_op_params.render_mode),
                                                      static_cast<float>(m_op_params.alpha_keypoint),
                                                      static_cast<float>(m_op_params.alpha_heat_map),
                                                      static_cast<float>(m_op_params.face_render_threshold)};
    m_openpose.configure(op_face_configuration);
  }

  m_openpose.setDefaultMaxSizeQueues(1);

  m_producer = std::unique_ptr<opw::Producer>(new opw::Producer());
  m_consumer = std::unique_ptr<opw::Consumer>(new opw::Consumer());
  m_consumer->configure(m_op_params.body_enable_detection,
                        m_op_params.body_model_name,
                        m_op_params.hands_enable_detection,
                        m_op_params.hands_model_name,
                        m_op_params.face_enable_detection,
                        m_op_params.face_model_name);
}

void open_ptrack::opw::Wrapper::compensateRoiOffsets(skeletons::types::SkeletonGroup& t_skeleton_group)
{
  for (auto& sk : t_skeleton_group.skeletons) {
    if (!open_ptrack::skeletons::utils::isNaN(sk.bounding_box.center.pose)) {
      sk.bounding_box.center.pose.position.setX(sk.bounding_box.center.pose.position.x() + m_region_of_interest.x);
      sk.bounding_box.center.pose.position.setY(sk.bounding_box.center.pose.position.y() + m_region_of_interest.y);
    }

    for (auto& mk : sk.markers) {
      if (mk.confidence > 0.0) {
        mk.center.pose.position.setX(mk.center.pose.position.x() + m_region_of_interest.x);
        mk.center.pose.position.setY(mk.center.pose.position.y() + m_region_of_interest.y);
      }
    }
  }
}

void open_ptrack::opw::Wrapper::removeUndesiredSkeletons(skeletons::types::SkeletonGroup& t_skeleton_group)
{
  // remove the skeletons with confidence < min_skeleton_confidence
  opw::commons::erase_if(t_skeleton_group.skeletons, [&](const auto& sk) {
    return (!std::isnan(sk.confidence) && sk.confidence < m_opw_params.min_skeleton_confidence);
  });
}

void open_ptrack::opw::Wrapper::removeUndesiredMarkers(skeletons::types::SkeletonGroup& t_skeleton_group)
{
  for (auto& sk : t_skeleton_group.skeletons) {
    // remove the markers contained in markers_to_discard
    for (const auto& marker_to_discard : m_opw_params.markers_to_discard) {
      sk.removeMarker(marker_to_discard);
    }

    // remove the markers with confidence < min_marker_confidence
    opw::commons::erase_if(sk.markers,
                           [&](const auto& mk) { return mk.confidence < m_opw_params.min_marker_confidence; });

    // update bounding box
    sk.bounding_box = open_ptrack::skeletons::utils::computeBoundingBox(sk);
  }
}

void open_ptrack::opw::Wrapper::addLinks(skeletons::types::SkeletonGroup& t_skeleton_group)
{
  double conf = std::numeric_limits<double>::quiet_NaN();
  open_ptrack::skeletons::types::Point pos(std::numeric_limits<double>::quiet_NaN(),
                                     std::numeric_limits<double>::quiet_NaN(),
                                     std::numeric_limits<double>::quiet_NaN());

  for (auto& sk : t_skeleton_group.skeletons) {
    sk.max_links = static_cast<unsigned int>(m_opw_params.links_config.size());

    for (auto& link_conf : m_opw_params.links_config) {
      if (sk.hasMarker(link_conf.parent_marker_id) && sk.hasMarker(link_conf.child_marker_id)) {
        auto& parent_marker = sk.getMarker(link_conf.parent_marker_id);
        auto& child_marker = sk.getMarker(link_conf.child_marker_id);

        conf = (parent_marker.confidence + child_marker.confidence) / 2;
        pos = parent_marker.center.pose.position.lerp(child_marker.center.pose.position, 0.5);

        sk.addLink(open_ptrack::skeletons::types::Link(
          link_conf.id, link_conf.parent_marker_id, link_conf.child_marker_id, conf, pos));
      }
    }
  }
}

void open_ptrack::opw::Wrapper::prepareOutputImg(const cv::Mat& t_full_size_img,
                                           const cv::Mat& t_cropped_img,
                                           cv::Mat& t_out_img)
{
  t_out_img = t_full_size_img;
  t_cropped_img.copyTo(t_out_img(m_region_of_interest));
  cv::rectangle(t_out_img, m_region_of_interest, cv::Scalar(0, 255, 0), 1);
}

void open_ptrack::opw::Wrapper::run(const sensor_msgs::ImageConstPtr& t_msg)
{
  ros::Time image_reception_time = ros::Time::now();

  if (m_opw_params.lag_minimization_enable && m_opw_params.max_input_delay > 0.0) {
    ros::Duration current_delay = image_reception_time - t_msg->header.stamp;
    if (current_delay.toSec() > m_opw_params.max_input_delay) {
      ROS_INFO_STREAM(m_node_namespace << " Lag Minimization: current delay " << current_delay.toSec()
                                       << " waiting for next sample");
      return;
    }
  }

  cv_bridge::CvImageConstPtr cv_ptr(cv_bridge::toCvShare(t_msg, sensor_msgs::image_encodings::BGR8));

  cv::Mat m;
  if (m_opw_params.use_region_of_interest) {
    m = cv_ptr->image(m_region_of_interest);
  }
  else {
    m = cv_ptr->image;
  }
  m_producer->setImage(op::Matrix(&m));

  opw::commons::framesDetectionsSharedPtr openpose_datum = m_producer->run();

  if (openpose_datum == nullptr) {
    ROS_WARN_STREAM(m_node_namespace << " Unable to create input for OpenPose. Skipping.");
    return;
  }

  bool success = m_openpose.waitAndEmplace(openpose_datum->front()->cvInputData);

  if (!success) {
    ROS_WARN_STREAM(m_node_namespace << " Unable to provide input to OpenPose. Skipping.");
    return;
  }

  success = m_openpose.waitAndPop(openpose_datum);

  if (!success) {
    ROS_WARN_STREAM(m_node_namespace << " Unable to get OpenPose output. Skipping.");
    return;
  }

  if (m_opw_params.publish_rendered_image_topic) {
    cv_bridge::CvImage out_cv_img;
    out_cv_img.header = t_msg->header;
    out_cv_img.encoding = sensor_msgs::image_encodings::BGR8;

    if (m_opw_params.use_region_of_interest) {
      prepareOutputImg(
        cv_ptr->image, *static_cast<cv::Mat*>(openpose_datum->front()->cvOutputData.getCvMat()), out_cv_img.image);
    }
    else {
      static_cast<cv::Mat*>(openpose_datum->front()->cvOutputData.getCvMat())->copyTo(out_cv_img.image);
    }

    m_out_img_pub.publish(out_cv_img.toImageMsg());
  }

  if (m_opw_params.publish_skeleton_group_topic) {
    m_consumer->setInputData(openpose_datum);
    std::unique_ptr<skeletons::types::SkeletonGroup> skeleton_group = m_consumer->processData();
    skeleton_group->time = ros::Time::now().toSec();
    skeleton_group->frame = t_msg->header.frame_id;

    for (auto& skeleton : skeleton_group->skeletons) {
      skeleton.src_time = t_msg->header.stamp.toSec();
      skeleton.src_frame = t_msg->header.frame_id;
    }

    removeUndesiredSkeletons(*skeleton_group);
    removeUndesiredMarkers(*skeleton_group);
    addLinks(*skeleton_group);

    if (m_opw_params.use_region_of_interest) {
      compensateRoiOffsets(*skeleton_group);
    }

    m_out_msg_pub.publish(open_ptrack::skeletons::utils::toMsg(*skeleton_group));
  }
}
