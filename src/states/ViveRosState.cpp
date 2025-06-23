#include <chrono>
#include <functional>

#include <mc_rtc/DataStore.h>

#include <HumanRetargetingController/HumanRetargetingController.h>
#include <HumanRetargetingController/states/ViveRosState.h>
#include <memory>
#include <rclcpp/executors.hpp>

using namespace HRC;

void ViveRosState::start(mc_control::fsm::Controller & _ctl)
{
  State::start(_ctl);

  // Setup ROS
  if(nh_)
  {
    mc_rtc::log::error("[ViveRosState] ROS node handle is already instantiated.");
    nh_.reset();
  }
  nh_ = rclcpp::Node::make_shared("ViveRosState");

  exectutor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  exectutor_->add_node(nh_);

  if(config_.has("configs") && config_("configs").has("joyTopics"))
  {
    for(const auto & joyConfig : config_("configs")("joyTopics"))
    {
      std::string topicName = joyConfig("topicName");
      std::string datastoreKey = joyConfig("datastoreKey");
      mc_rtc::log::info("[ViveRosState] Subscribe the {} topic and save it to the {} key in the datastore.", topicName,
                        datastoreKey);
      // TODO fix this line
      //joySubList_.push_back(nh_->create_subscription<sensor_msgs::msg::Joy>(topicName, 1, std::bind(&ViveRosState::joyCallback, this, std::placeholders::_1, datastoreKey)));
    }
  }

  output("OK");
}

bool ViveRosState::run(mc_control::fsm::Controller &)
{
  // Call ROS callback
  exectutor_->spin_some(std::chrono::nanoseconds(0));

  return false;
}

void ViveRosState::teardown(mc_control::fsm::Controller &)
{
  for(auto & joySub : joySubList_)
  {
    // TODO check what is the correct thing to do here
    //joySub.shutdown();
  }

  nh_.reset();
}

void ViveRosState::joyCallback(const sensor_msgs::msg::Joy & joyMsg, const std::string & datastoreKey)
{
  setDatastore<sensor_msgs::msg::Joy>(ctl().datastore(), datastoreKey, joyMsg);
}

EXPORT_SINGLE_STATE("HRC::ViveRos", ViveRosState)
