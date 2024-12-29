#include <functional>

#include <mc_rtc/DataStore.h>

#include <HumanRetargetingController/HumanRetargetingController.h>
#include <HumanRetargetingController/states/ViveRosState.h>

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
  nh_ = std::make_shared<ros::NodeHandle>();
  // Use a dedicated queue so as not to call callbacks of other modules
  nh_->setCallbackQueue(&callbackQueue_);

  if(config_.has("configs") && config_("configs").has("joyTopics"))
  {
    for(const auto & joyConfig : config_("configs")("joyTopics"))
    {
      std::string topicName = joyConfig("topicName");
      std::string datastoreKey = joyConfig("datastoreKey");
      mc_rtc::log::info("[ViveRosState] Subscribe the {} topic and save it to the {} key in the datastore.", topicName,
                        datastoreKey);
      joySubList_.push_back(nh_->subscribe<sensor_msgs::Joy>(
          topicName, 1, std::bind(&ViveRosState::joyCallback, this, std::placeholders::_1, datastoreKey)));
    }
  }

  output("OK");
}

bool ViveRosState::run(mc_control::fsm::Controller &)
{
  // Call ROS callback
  callbackQueue_.callAvailable(ros::WallDuration());

  return false;
}

void ViveRosState::teardown(mc_control::fsm::Controller &)
{
  for(auto & joySub : joySubList_)
  {
    joySub.shutdown();
  }

  nh_.reset();
}

void ViveRosState::joyCallback(const sensor_msgs::Joy::ConstPtr & joyMsg, const std::string & datastoreKey)
{
  setDatastore<sensor_msgs::Joy>(ctl().datastore(), datastoreKey, *joyMsg);
}

template<class ValueType>
void ViveRosState::setDatastore(mc_rtc::DataStore & datastore, const std::string & key, const ValueType & value)
{
  if(datastore.has(key))
  {
    datastore.assign<ValueType>(key, value);
  }
  else
  {
    datastore.make<ValueType>(key, value);
  }
}

EXPORT_SINGLE_STATE("HRC::ViveRos", ViveRosState)
