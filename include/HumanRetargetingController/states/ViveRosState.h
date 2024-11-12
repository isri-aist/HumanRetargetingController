#pragma once

#include <HumanRetargetingController/State.h>

#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

namespace HRC
{
/** \brief FSM state to initialize. */
struct ViveRosState : State
{
public:
  /** \brief Start. */
  void start(mc_control::fsm::Controller & ctl) override;

  /** \brief Run. */
  bool run(mc_control::fsm::Controller & ctl) override;

  /** \brief Teardown. */
  void teardown(mc_control::fsm::Controller & ctl) override;

protected:
  /** \brief ROS callback of joy topic. */
  void joyCallback(const sensor_msgs::Joy::ConstPtr & joyMsg, const std::string & datastoreKey);

  /** \brief Set datastore entry. */
  template<class ValueType>
  void setDatastore(mc_rtc::DataStore & datastore, const std::string & key, const ValueType & value);

protected:
  //! ROS node handle
  std::shared_ptr<ros::NodeHandle> nh_;

  //! ROS callback queue
  ros::CallbackQueue callbackQueue_;

  //! ROS subscriber of joy topics
  std::vector<ros::Subscriber> joySubList_;
};
} // namespace HRC
