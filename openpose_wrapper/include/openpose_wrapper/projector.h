#ifndef open_ptrack_opw3d_projector_h
#define open_ptrack_opw3d_projector_h

// ROS dependencies
#include <ros/ros.h>
#include <tf/transform_listener.h>

// ROS External Packages dependencies
#include <sensor_msgs/CameraInfo.h>

#include "opencv2/core/mat.hpp"
#include <memory>

#include "openpose_wrapper/SkeletonGroup.h"
#include "openpose_wrapper/queue.h"
#include "openpose_wrapper/skeletons/types.h"

#define BASH_MSG_RESET "\033[0m"
#define BASH_MSG_GREEN "\033[32m"

namespace open_ptrack {
  namespace opw3d {

    struct Opw3dProjectorParams
    {
      std::string node_name = "";

      std::string in_skeleton_group_topic = "";
      std::string in_depth_topic = "";
      std::string in_camera_info_topic = "";

      std::string out_msg_topic_name = "3d_skeleton_group";

      bool publish_in_global_reference_frame = false;
      std::string global_reference_frame = "world";

      double depth_to_meters_multiplier = 1;

      bool enable_bounding_box = false;
      double epsilon_time_matching = 0.001;
    };

    class Projector
    {
    public:
      Projector();

      void configure();
      void start();
      void stop();

    private:
      struct Opw3dProjectorPrivateParams
      {
        bool tf_configured = false;
        std::string local_reference_frame = "";
        std::string output_reference_frame = "";

        bool camera_info_configured = false;
        bool depth_configured = false;
        unsigned int width = 0;
        unsigned int height = 0;
        std::array<double, 9> camera_info_k_matrix = {0};

        const double& fx = camera_info_k_matrix.at(0);
        const double& fy = camera_info_k_matrix.at(4);
        const double& cx = camera_info_k_matrix.at(2);
        const double& cy = camera_info_k_matrix.at(5);

        // depth averaging parameters
        double bb_equivalent_side_scaling_factor = 0.01;
        double valid_to_total_mks_scaling_factor = 3.0;
      };

      void setupRosTopics();
      void configureGlobalFrameConverter();
      std::string extractImageQualityFromTopicString(const std::string& t_topic_name) const;

      // Callbacks
      void pushDepth(sensor_msgs::ImageConstPtr t_depth);
      void setCameraInfo(sensor_msgs::CameraInfoConstPtr t_camera_info);
      void to3dCallback(openpose_wrapper::SkeletonGroupConstPtr t_skeleton_group_msg);

      skeletons::types::SkeletonGroup project(openpose_wrapper::SkeletonGroupConstPtr t_skeleton_group,
                                              const cv::Mat& t_depth_mat);

      double toMeters(const double& t_x) const;
      std::vector<double> toStdVector(const cv::Mat& t_mat) const;
      cv::Rect computeDepthAveragingArea(const open_ptrack::skeletons::types::Skeleton& t_skeleton);
      double getDepthMeanValue(const cv::Mat& t_roi) const;
      double getDepthMedianValue(const cv::Mat& t_roi) const;
      void toGlobalReferenceFrame(open_ptrack::skeletons::types::KinematicState& t_ks) const;

      template <typename T>
      T inRange(const T& from, const T& val, const T& to)
      {
        return std::min(std::max(from, val), to);
      }

      ros::NodeHandle m_nh;
      std::string m_node_namespace;

      ros::Subscriber m_in_depth_sub;
      ros::Subscriber m_in_camera_info_sub;
      ros::Subscriber m_in_skeleton_group_sub;
      ros::Publisher m_out_msg_pub;

      tf::TransformListener m_tf_listener;

      std::unique_ptr<opw3d::Queue> m_depth_queue = nullptr;

      Opw3dProjectorParams m_params{};
      Opw3dProjectorPrivateParams m_private_params{};
      bool m_configured = false;
    };
  } // namespace opw3d
} // namespace open_ptrack

#endif
