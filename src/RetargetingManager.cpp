#include <limits>

#include <mc_tasks/ImpedanceTask.h>

#include <HumanRetargetingController/HumanRetargetingController.h>
#include <HumanRetargetingController/RetargetingManagerSet.h>

using namespace HRC;

void RetargetingManager::Configuration::load(const mc_rtc::Configuration & mcRtcConfig)
{
  mcRtcConfig("name", name);
  mcRtcConfig("bodyPart", bodyPart);
  mcRtcConfig("bodyGroups", bodyGroups);
  mcRtcConfig("targetPoseTopicName", targetPoseTopicName);
  if(targetPoseTopicName.empty())
  {
    targetPoseTopicName = "/hrc/poses/" + bodyPart;
  }
  if(mcRtcConfig.has("stiffness"))
  {
    if(mcRtcConfig("stiffness").isNumeric())
    {
      stiffness = Eigen::Vector6d::Constant(mcRtcConfig("stiffness"));
    }
    else
    {
      stiffness = mcRtcConfig("stiffness");
    }
  }
}

RetargetingManager::RetargetingManager(HumanRetargetingController * ctlPtr, const mc_rtc::Configuration & mcRtcConfig)
: ctlPtr_(ctlPtr)
{
  config_.load(mcRtcConfig);
}

void RetargetingManager::reset()
{
  isTaskEnabled_ = false;
  humanTargetPose_ = std::nullopt;
  stiffnessRatioFunc_ = nullptr;

  // Setup ROS
  targetPoseSub_ = nh()->subscribe<geometry_msgs::PoseStamped>(config_.targetPoseTopicName, 1,
                                                               &RetargetingManager::poseCallback, this);
}

void RetargetingManager::update()
{
  if(!isTaskEnabled_)
  {
    return;
  }

  sva::PTransformd robotTargetPose = humanTargetPose_.value() * humanBasePose().value().inv() * robotBasePose();
  retargetingTask()->targetPose(robotTargetPose);
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
  targetPoseSub_.shutdown();

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
    mc_rtc::log::warning("[RetargetingManager({})] Task is already enabled.", config_.bodyPart);
    return;
  }

  isTaskEnabled_ = true;

  retargetingTask()->reset();
  ctl().solver().addTask(retargetingTask());
  retargetingTask()->stiffness(0.0);

  constexpr double stiffnessInterpDuration = 2.0; // [sec]
  stiffnessRatioFunc_ = std::make_shared<TrajColl::CubicInterpolator<double>>(
      std::map<double, double>{{ctl().t(), 0.0}, {ctl().t() + stiffnessInterpDuration, 1.0}});
}

void RetargetingManager::disableTask()
{
  if(!isTaskEnabled_)
  {
    mc_rtc::log::warning("[RetargetingManager({})] Task is already disabled.", config_.bodyPart);
    return;
  }

  isTaskEnabled_ = false;

  ctl().solver().removeTask(retargetingTask());
  retargetingTask()->stiffness(0.0);
}

std::shared_ptr<ros::NodeHandle> RetargetingManager::nh() const
{
  return ctl().retargetingManagerSet_->nh_;
}

const std::optional<sva::PTransformd> & RetargetingManager::humanBasePose() const
{
  return ctl().retargetingManagerSet_->humanBasePose_;
}

const sva::PTransformd & RetargetingManager::robotBasePose() const
{
  return ctl().retargetingManagerSet_->robotBasePose_;
}

const std::shared_ptr<mc_tasks::force::ImpedanceTask> & RetargetingManager::retargetingTask() const
{
  return ctl().retargetingTasks_.at(config_.bodyPart);
}

void RetargetingManager::poseCallback(const geometry_msgs::PoseStamped::ConstPtr & poseStMsg)
{
  const auto & poseMsg = poseStMsg->pose;
  humanTargetPose_ = sva::PTransformd(
      Eigen::Quaterniond(poseMsg.orientation.w, poseMsg.orientation.x, poseMsg.orientation.y, poseMsg.orientation.z)
          .normalized()
          .toRotationMatrix()
          .transpose(),
      Eigen::Vector3d(poseMsg.position.x, poseMsg.position.y, poseMsg.position.z));
}
