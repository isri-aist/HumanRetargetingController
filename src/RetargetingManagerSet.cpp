#include <mc_rtc/gui/ArrayInput.h>
#include <mc_rtc/gui/Label.h>

#include <HumanRetargetingController/HumanRetargetingController.h>
#include <HumanRetargetingController/RetargetingManagerSet.h>

using namespace HRC;

void RetargetingManagerSet::Configuration::load(const mc_rtc::Configuration & mcRtcConfig)
{
  mcRtcConfig("name", name);
  mcRtcConfig("baseFrame", baseFrame);
  mcRtcConfig("basePoseTopicName", basePoseTopicName);
}

RetargetingManagerSet::RetargetingManagerSet(HumanRetargetingController * ctlPtr,
                                             const mc_rtc::Configuration & mcRtcConfig)
: ctlPtr_(ctlPtr)
{
  config_.load(mcRtcConfig);

  for(const mc_rtc::Configuration & retargetingManagerConfig : mcRtcConfig("RetargetingManagerList"))
  {
    this->emplace(retargetingManagerConfig("bodyPart"),
                  std::make_shared<RetargetingManager>(ctlPtr, retargetingManagerConfig));
  }
}

void RetargetingManagerSet::reset()
{
  isTaskEnabled_ = false;
  humanBasePose_ = std::nullopt;
  robotBasePose_ = sva::PTransformd::Identity();

  // Setup ROS
  if(nh_)
  {
    mc_rtc::log::error("[RetargetingManagerSet] ROS node handle is already instantiated.");
  }
  else
  {
    nh_ = std::make_shared<ros::NodeHandle>();
  }

  // Use a dedicated queue so as not to call callbacks of other modules
  nh_->setCallbackQueue(&callbackQueue_);
  basePoseSub_ = nh_->subscribe<geometry_msgs::PoseStamped>(config_.basePoseTopicName, 1,
                                                            &RetargetingManagerSet::basePoseCallback, this);

  for(const auto & limbManagerKV : *this)
  {
    limbManagerKV.second->reset();
  }
}

void RetargetingManagerSet::update()
{
  // Call ROS callback
  callbackQueue_.callAvailable(ros::WallDuration());

  robotBasePose_ = ctl().robot().frame(config_.baseFrame).position();

  for(const auto & limbManagerKV : *this)
  {
    limbManagerKV.second->update();
  }

  // Trigger the task to be enabled
  if(!isTaskEnabled_)
  {
    bool isHumanPoseReady = humanBasePose_.has_value();
    for(const auto & limbManagerKV : *this)
    {
      if(!isHumanPoseReady)
      {
        break;
      }
      if(!limbManagerKV.second->humanTargetPose_.has_value())
      {
        isHumanPoseReady = false;
      }
    }

    if(isHumanPoseReady)
    {
      isTaskEnabled_ = true;
      mc_rtc::log::success("[RetargetingManagerSet] Enable retargeting tasks.");

      for(const auto & limbManagerKV : *this)
      {
        limbManagerKV.second->enableTask();
      }
    }
  }
}

void RetargetingManagerSet::stop()
{
  basePoseSub_.shutdown();
  nh_.reset();

  removeFromGUI(*ctl().gui());
  removeFromLogger(ctl().logger());

  for(const auto & limbManagerKV : *this)
  {
    limbManagerKV.second->stop();
  }
}

void RetargetingManagerSet::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  for(const auto & limbManagerKV : *this)
  {
    limbManagerKV.second->addToGUI(gui);
  }

  // TODO
}

void RetargetingManagerSet::removeFromGUI(mc_rtc::gui::StateBuilder & gui)
{
  gui.removeCategory({ctl().name(), config_.name});

  // GUI of each RetargetingManager is not removed here (removed via stop method)
}

void RetargetingManagerSet::addToLogger(mc_rtc::Logger & logger)
{
  for(const auto & limbManagerKV : *this)
  {
    limbManagerKV.second->addToLogger(logger);
  }

  // TODO
}

void RetargetingManagerSet::removeFromLogger(mc_rtc::Logger & // logger
)
{
  // Log of each RetargetingManager is not removed here (removed via stop method)
}

void RetargetingManagerSet::basePoseCallback(const geometry_msgs::PoseStamped::ConstPtr & poseStMsg)
{
  const auto & poseMsg = poseStMsg->pose;
  humanBasePose_ = sva::PTransformd(
      Eigen::Quaterniond(poseMsg.orientation.w, poseMsg.orientation.x, poseMsg.orientation.y, poseMsg.orientation.z)
          .normalized()
          .toRotationMatrix()
          .transpose(),
      Eigen::Vector3d(poseMsg.position.x, poseMsg.position.y, poseMsg.position.z));
}
