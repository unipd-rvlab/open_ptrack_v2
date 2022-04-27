#ifndef open_ptrack_opw_wrapper_h
#define open_ptrack_opw_wrapper_h

// ROS dependencies
#include <ros/ros.h>

// ROS External Packages dependencies
#include <image_transport/image_transport.h>

// OpenCV dependencies
#include <opencv2/core.hpp>

// OpenPose dependencies
#include <openpose/wrapper/wrapper.hpp>

// Internal dependencies
#include "openpose_wrapper/consumer.h"
#include "openpose_wrapper/producer.h"

namespace open_ptrack {
  namespace opw {

    struct LinkProperties
    {
      int id = -1;
      int parent_marker_id = -1;
      int child_marker_id = -1;
    };

    struct WrapperParameters
    {
      std::string in_image_topic = "";
      std::string out_image_topic_name = "openpose_detection";
      std::string out_msg_topic_name = "skeleton_group";

      bool publish_rendered_image_topic = false;
      bool publish_skeleton_group_topic = false;

      bool use_region_of_interest = false;
      int roi_top_left_corner_x = 0;
      int roi_top_left_corner_y = 0;
      int roi_width = 0;
      int roi_height = 0;

      double min_skeleton_confidence = 0.0;
      double min_marker_confidence = 0.0;
      bool enable_marker_filtering = false;

      std::vector<int> markers_to_discard;
      std::vector<LinkProperties> links_config;

      bool lag_minimization_enable = false;
      double max_input_delay = 0.0;
    };

    struct OpenPoseParameters
    {
      //----------------------------
      // Generic OpenPose parameters
      //----------------------------
      int number_of_gpus = -1;
      int gpu_number_start = 0;

      double max_fps = -1;
      double upsampling_ratio = 0.0;

      std::string models_path = "models/";
      std::string output_size = "-1x-1";

      int keypoints_scale_mode = 0;
      int max_number_of_people = -1;

      int render_mode = -1;
      double alpha_keypoint = 0.6;
      double alpha_heat_map = 0.7;
      bool blend_original_frame = true;

      bool add_part_candidates = false;
      bool maximize_positives = false;
      bool enable_google_logging = true;

      //-------------------------
      // Body detector parameters
      //-------------------------
      bool body_enable_detection = true;
      std::string body_model_name = "BODY_25";
      std::string body_net_input_size = "-1x368";
      int body_scales_number = 1;
      double body_scale_gap = 0.25;
      int body_default_parts_to_render = 0;
      bool body_heat_maps_add_pafs = false;
      bool body_heat_maps_add_parts = false;
      bool body_heat_maps_add_background = false;
      int body_heat_maps_scale_mode = 4;
      double body_render_threshold = 0.05;

      //--------------------------
      // Hands detector parameters
      //--------------------------
      bool hands_enable_detection = false;
      std::string hands_model_name = "HAND";
      int hands_locations_detector = 0;
      int hands_scales_number = 1;
      double hands_scale_range = 0.4;
      double hands_render_threshold = 0.2;
      std::string hands_net_input_size = "368x368";

      //-------------------------
      // Face detector parameters
      //-------------------------
      bool face_enable_detection = false;
      std::string face_model_name = "FACE";
      int face_location_detector = 0;
      double face_render_threshold = 0.4;
      std::string face_net_input_size = "368x368";
    };

    //===============================
    // OpenPoseWrapper: Wrapper Class
    //===============================
    class Wrapper
    {
    public:
      //---------------------------
      // Constructors / Destructors
      //---------------------------
      Wrapper();
      ~Wrapper();

      //------------------------
      // Custom public functions
      //------------------------
      void configure();

      void start();
      void stop();
      void run(const sensor_msgs::ImageConstPtr& t_msg);

    private:
      //------------------
      // Private functions
      //------------------
      void configureOpenPose();
      void configureWrapper();

      void removeUndesiredSkeletons(skeletons::types::SkeletonGroup& t_skeleton_group);
      void removeUndesiredMarkers(skeletons::types::SkeletonGroup& t_skeleton_group);
      void addLinks(skeletons::types::SkeletonGroup& t_skeleton_group);

      void prepareOutputImg(const cv::Mat& t_full_size_img, const cv::Mat& t_cropped_img, cv::Mat& t_out_img);

      void compensateRoiOffsets(skeletons::types::SkeletonGroup& t_skeleton_group);

      void checkInOutConfiguration();
      void setupRosTopics();

      //-------------------------
      // Private member variables
      //-------------------------
      ros::NodeHandle m_nh;
      std::string m_node_namespace;
      image_transport::ImageTransport m_ith;
      image_transport::Subscriber m_in_img_sub;
      image_transport::Publisher m_out_img_pub;
      ros::Publisher m_out_msg_pub;

      OpenPoseParameters m_op_params{};
      WrapperParameters m_opw_params{};

      cv::Rect m_region_of_interest = {};

      std::unique_ptr<opw::Producer> m_producer = nullptr;
      std::unique_ptr<opw::Consumer> m_consumer = nullptr;

      op::WrapperT<op::Datum> m_openpose{op::ThreadManagerMode::Asynchronous};

      bool m_opw_configured = false;
    };
  } // namespace opw
} // namespace open_ptrack

#endif
