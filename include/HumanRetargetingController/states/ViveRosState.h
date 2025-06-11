#pragma once

#include <HumanRetargetingController/State.h>

#include <rclcpp/executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/detail/joy__struct.hpp>
#include <sensor_msgs/msg/joy.hpp>

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
  void joyCallback(const sensor_msgs::msg::Joy & joyMsg, const std::string & datastoreKey);

  /** \brief Set datastore entry. */
  template<class ValueType>
  void setDatastore(mc_rtc::DataStore & datastore, const std::string & key, const ValueType & value);

protected:
  //! ROS node handle
  rclcpp::Node::SharedPtr nh_;

  //! ROS executor
  rclcpp::Executor::SharedPtr exectutor_;

  //! ROS subscriber of joy topics
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr> joySubList_;
};
} // namespace HRC
