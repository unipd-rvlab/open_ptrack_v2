#include "openpose_wrapper/queue.h"

open_ptrack::opw3d::Queue::Queue(double dt_epsilon)
  : DT_EPSILON(dt_epsilon){};

void open_ptrack::opw3d::Queue::push_back(sensor_msgs::ImageConstPtr t_msg)
{
  m_queue.push_back(t_msg);
}

sensor_msgs::ImageConstPtr open_ptrack::opw3d::Queue::get(const ros::Time& t_time)
{
  for (auto& el : m_queue) {
    ros::Duration time_diff(el.get()->header.stamp.toSec() - t_time.toSec());
    if (time_diff.toSec() > ros::Duration(-DT_EPSILON).toSec()
        && time_diff.toSec() < ros::Duration(DT_EPSILON).toSec()) {
      return el;
    }
  }
  return nullptr;
}

void open_ptrack::opw3d::Queue::clear_until_time(const ros::Time& t_time)
{
  m_queue.erase(
    std::remove_if(m_queue.begin(),
                   m_queue.end(),
                   [&](sensor_msgs::ImageConstPtr p) { return (p->header.stamp - t_time).toSec() < DT_EPSILON; }),
    m_queue.end());
}

size_t open_ptrack::opw3d::Queue::get_size()
{
  return m_queue.size();
}
