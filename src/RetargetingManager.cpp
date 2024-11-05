#include <limits>

#include <mc_tasks/ImpedanceTask.h>

#include <HumanRetargetingController/RetargetingManagerSet.h>
#include <HumanRetargetingController/HumanRetargetingController.h>

using namespace HRC;

void RetargetingManager::Configuration::load(const mc_rtc::Configuration & mcRtcConfig)
{
  mcRtcConfig("name", name);
  mcRtcConfig("bodyPart", bodyPart);
  mcRtcConfig("bodyGroups", bodyGroups);
  mcRtcConfig("poseTopicName", poseTopicName);
  mcRtcConfig("stiffness", stiffness);
}

RetargetingManager::RetargetingManager(HumanRetargetingController * ctlPtr, const mc_rtc::Configuration & mcRtcConfig)
: ctlPtr_(ctlPtr)
{
  config_.load(mcRtcConfig);
}

void RetargetingManager::reset()
{
  isFirstMsgObtained_ = false;
  isTaskEnabled_ = false;

  // Setup ROS
  if(nh_)
  {
    mc_rtc::log::error("[RetargetingManager({})] ROS node handle is already instantiated.", config_.bodyPart);
  }
  else
  {
    nh_ = std::make_shared<ros::NodeHandle>();
  }

  // Use a dedicated queue so as not to call callbacks of other modules
  nh_->setCallbackQueue(&callbackQueue_);
  poseSub_ = nh_->subscribe<geometry_msgs::PoseStamped>(config_.poseTopicName, 1, &RetargetingManager::poseCallback, this);
}

void RetargetingManager::update()
{
  // Call ROS callback
  callbackQueue_.callAvailable(ros::WallDuration());

  if(!isTaskEnabled_)
  {
    return;
  }

  retargetingTask()->targetPose(targetPose_);
  retargetingTask()->targetVel(sva::MotionVecd::Zero());
  retargetingTask()->targetAccel(sva::MotionVecd::Zero());

  // Interpolate task stiffness
  if(stiffnessRatioFunc_)
  {
    if(ctl().t() <= stiffnessRatioFunc_->endTime())
    {
      double stiffnessRatio = (*stiffnessRatioFunc_)(ctl().t());
      retargetingTask()->stiffness(stiffnessRatio * config_.stiffness);
    }
    else
    {
      retargetingTask()->stiffness(Eigen::VectorXd(config_.stiffness));
      stiffnessRatioFunc_.reset();
    }
  }
}

void RetargetingManager::stop()
{
  poseSub_.shutdown();
  nh_.reset();

  ctl().solver().removeTask(retargetingTask());

  removeFromGUI(*ctl().gui());
  removeFromLogger(ctl().logger());
}

void RetargetingManager::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  // TODO
}

void RetargetingManager::removeFromGUI(mc_rtc::gui::StateBuilder & gui)
{
  gui.removeCategory({ctl().name(), config_.name, config_.bodyPart});
}

void RetargetingManager::addToLogger(mc_rtc::Logger & logger)
{
  // TODO
}

void RetargetingManager::removeFromLogger(mc_rtc::Logger & logger)
{
  logger.removeLogEntries(this);
}

void RetargetingManager::enableTask()
{
  if(isTaskEnabled_)
  {
    mc_rtc::log::warning("[RetargetingManager({})] Task is already enabled.");
    return;
  }

  isTaskEnabled_ = true;

  retargetingTask()->reset();
  ctl().solver().addTask(retargetingTask());
  retargetingTask()->stiffness(0.0);

  constexpr double stiffnessInterpDuration = 10.0; // [sec]
  stiffnessRatioFunc_ = std::make_shared<TrajColl::CubicInterpolator<double>>(
      std::map<double, double>{{ctl().t(), 0.0}, {ctl().t() + stiffnessInterpDuration, 1.0}});
}

void RetargetingManager::disableTask()
{
  if(!isTaskEnabled_)
  {
    mc_rtc::log::warning("[RetargetingManager({})] Task is already disabled.");
    return;
  }

  isTaskEnabled_ = false;

  ctl().solver().removeTask(retargetingTask());
  retargetingTask()->stiffness(0.0);
}

const std::shared_ptr<mc_tasks::force::ImpedanceTask> & RetargetingManager::retargetingTask() const
{
  return ctl().retargetingTasks_.at(config_.bodyPart);
}

void RetargetingManager::poseCallback(const geometry_msgs::PoseStamped::ConstPtr & poseStMsg)
{
  if(!isFirstMsgObtained_)
  {
    isFirstMsgObtained_ = true;
  }

  const auto & poseMsg = poseStMsg->pose;
  targetPose_ = sva::PTransformd(
      Eigen::Quaterniond(poseMsg.orientation.w, poseMsg.orientation.x, poseMsg.orientation.y, poseMsg.orientation.z)
      .normalized()
      .toRotationMatrix()
      .transpose(),
      Eigen::Vector3d(poseMsg.position.x, poseMsg.position.y, poseMsg.position.z));
}
