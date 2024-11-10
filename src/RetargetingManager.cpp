#include <mc_rtc/gui/Label.h>
#include <mc_rtc/gui/Point3D.h>
#include <mc_tasks/ImpedanceTask.h>
#include <mc_tasks/TransformTask.h>

#include <HumanRetargetingController/HumanRetargetingController.h>
#include <HumanRetargetingController/RetargetingManagerSet.h>

using namespace HRC;

void RetargetingManager::Configuration::load(const mc_rtc::Configuration & mcRtcConfig)
{
  mcRtcConfig("name", name);
  mcRtcConfig("bodyPart", bodyPart);
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
  mcRtcConfig("targetPoseExpirationDuration", targetPoseExpirationDuration);
}

RetargetingManager::RetargetingManager(HumanRetargetingController * ctlPtr, const mc_rtc::Configuration & mcRtcConfig)
: ctlPtr_(ctlPtr)
{
  config_.load(mcRtcConfig);
}

void RetargetingManager::reset()
{
  retargetingPhase_ = RetargetingPhase::Disabled;
  humanTargetPose_ = std::nullopt;
  robotTargetPose_ = std::nullopt;
  targetPoseLatestTime_ = -1;
  stiffnessRatioFunc_ = nullptr;

  // Setup ROS
  targetPoseSub_ = nh()->subscribe<geometry_msgs::PoseStamped>(config_.targetPoseTopicName, 1,
                                                               &RetargetingManager::targetPoseCallback, this);
}

void RetargetingManager::preUpdate()
{
  // Update validity
  updateValidity();
}

void RetargetingManager::postUpdate()
{
  // Update target pose
  if(humanTargetPose_.has_value() && humanBasePose().has_value())
  {
    robotTargetPose_ = humanTargetPose_.value() * humanBasePose().value().inv() * robotBasePose();
  }

  // Update task target
  if(retargetingPhase_ == RetargetingPhase::Enabled)
  {
    if(retargetingImpTask())
    {
      retargetingImpTask()->targetPose(robotTargetPose_.value());
      retargetingImpTask()->targetVel(sva::MotionVecd::Zero());
      retargetingImpTask()->targetAccel(sva::MotionVecd::Zero());
    }
    else
    {
      retargetingTask()->target(robotTargetPose_.value());
      retargetingTask()->targetVel(sva::MotionVecd::Zero());
    }
  }

  // Interpolate task stiffness
  if(stiffnessRatioFunc_)
  {
    if(ctl().t() < stiffnessRatioFunc_->endTime())
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

  // Update GUI marker
  ctl().gui()->removeCategory({ctl().name(), config_.name, config_.bodyPart, "Marker"});
  if(robotTargetPose_.has_value())
  {
    ctl().gui()->addElement({ctl().name(), config_.name, config_.bodyPart, "Marker"},
                            mc_rtc::gui::Point3D("TargetPoint",
                                                 mc_rtc::gui::PointConfig(mc_rtc::gui::Color(0, 1, 0, 0.5), 0.15),
                                                 [this]() { return robotTargetPose_.value().translation(); }));
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
  gui.addElement({ctl().name(), config_.name, config_.bodyPart, "Status"},
                 mc_rtc::gui::Label("RetargetingPhase", [this]() { return std::to_string(retargetingPhase_); }));
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
  if(retargetingPhase_ == RetargetingPhase::Enabled)
  {
    mc_rtc::log::error("[RetargetingManager({})] Task is already enabled.", config_.bodyPart);
    return;
  }

  if(retargetingPhase_ == RetargetingPhase::Disabled)
  {
    retargetingTask()->reset();
    ctl().solver().addTask(retargetingTask());
    retargetingTask()->stiffness(0.0);

    constexpr double stiffnessInterpDuration = 2.0; // [sec]
    stiffnessRatioFunc_ = std::make_shared<TrajColl::CubicInterpolator<double>>(
        std::map<double, double>{{ctl().t(), 0.0}, {ctl().t() + stiffnessInterpDuration, 1.0}});
  }

  retargetingPhase_ = RetargetingPhase::Enabled;
}

void RetargetingManager::disableTask()
{
  if(retargetingPhase_ == RetargetingPhase::Disabled)
  {
    mc_rtc::log::error("[RetargetingManager({})] Task is already disabled.", config_.bodyPart);
    return;
  }

  ctl().solver().removeTask(retargetingTask());

  stiffnessRatioFunc_ = nullptr;

  retargetingPhase_ = RetargetingPhase::Disabled;
}

void RetargetingManager::freezeTask()
{
  if(retargetingPhase_ == RetargetingPhase::Frozen)
  {
    mc_rtc::log::error("[RetargetingManager({})] Task is already frozen.", config_.bodyPart);
    return;
  }
  else if(retargetingPhase_ == RetargetingPhase::Disabled)
  {
    mc_rtc::log::error("[RetargetingManager({})] Task cannot be frozen from the disabled phase.", config_.bodyPart);
    return;
  }

  retargetingPhase_ = RetargetingPhase::Frozen;
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

const std::shared_ptr<mc_tasks::TransformTask> & RetargetingManager::retargetingTask() const
{
  return ctl().retargetingTasks_.at(config_.bodyPart);
}

const std::shared_ptr<mc_tasks::force::ImpedanceTask> RetargetingManager::retargetingImpTask() const
{
  return std::dynamic_pointer_cast<mc_tasks::force::ImpedanceTask>(retargetingTask());
}

void RetargetingManager::updateValidity()
{
  bool isValid = true;

  if(isValid && !humanTargetPose_.has_value())
  {
    isValid = false;
  }

  if(isValid && (targetPoseLatestTime_ < ctl().t() - config_.targetPoseExpirationDuration))
  {
    isValid = false;
  }

  if(!isValid)
  {
    humanTargetPose_ = std::nullopt;
    robotTargetPose_ = std::nullopt;
  }
}

void RetargetingManager::targetPoseCallback(const geometry_msgs::PoseStamped::ConstPtr & poseStMsg)
{
  const auto & poseMsg = poseStMsg->pose;
  humanTargetPose_ = sva::PTransformd(
      Eigen::Quaterniond(poseMsg.orientation.w, poseMsg.orientation.x, poseMsg.orientation.y, poseMsg.orientation.z)
          .normalized()
          .toRotationMatrix()
          .transpose(),
      Eigen::Vector3d(poseMsg.position.x, poseMsg.position.y, poseMsg.position.z));
  targetPoseLatestTime_ = ctl().t();
}
