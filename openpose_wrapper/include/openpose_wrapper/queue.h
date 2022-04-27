#ifndef open_ptrack_opw3d_queue_h
#define open_ptrack_opw3d_queue_h

#include "ros/ros.h"
#include "ros/time.h"
#include "sensor_msgs/Image.h"

namespace open_ptrack {
  namespace opw3d {

    class Queue
    {
    public:
      Queue(double dt_epsilon = 0.001);

      void push_back(sensor_msgs::ImageConstPtr t_msg);
      sensor_msgs::ImageConstPtr get(const ros::Time& t_time);

      void clear_until_time(const ros::Time& t_time);

      size_t get_size();

    private:
      const double DT_EPSILON;
      std::vector<sensor_msgs::ImageConstPtr> m_queue;
    };
  } // namespace opw3d

} // namespace open_ptrack
#endif
