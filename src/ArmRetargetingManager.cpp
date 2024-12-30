#include <mc_rtc/gui/Button.h>
#include <mc_rtc/gui/Label.h>
#include <mc_tasks/ImpedanceTask.h>
#include <mc_tasks/TransformTask.h>

#include <HumanRetargetingController/ArmRetargetingManager.h>
#include <HumanRetargetingController/HumanRetargetingController.h>
#include <HumanRetargetingController/RetargetingManagerSet.h>
#include <HumanRetargetingController/RosPoseManager.h>

using namespace HRC;

void ArmRetargetingManager::Configuration::load(const mc_rtc::Configuration & mcRtcConfig)
{
  armSide = strToArmSide(mcRtcConfig("armSide"));

  humanElbowPoseTopicName = static_cast<std::string>(mcRtcConfig("humanElbowPoseTopicName"));
  humanWristPoseTopicName = static_cast<std::string>(mcRtcConfig("humanWristPoseTopicName"));

  elbowTaskName = static_cast<std::string>(mcRtcConfig("elbowTaskName"));
  wristTaskName = static_cast<std::string>(mcRtcConfig("wristTaskName"));

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

  mcRtcConfig("robotCalibPostures", robotCalibPostures);

  mcRtcConfig("calibResultConfig", calibResultConfig);
}

ArmRetargetingManager::CalibResult::CalibResult()
{
  reset();
}

void ArmRetargetingManager::CalibResult::reset()
{
  isInitialized = false;

  humanTransFromBaseToShoulder = sva::PTransformd::Identity();
  robotTransFromBaseToShoulder = sva::PTransformd::Identity();

  elbowRotTransFromHumanToRobot = sva::PTransformd::Identity();
  wristRotTransFromHumanToRobot = sva::PTransformd::Identity();

  elbowScale = 1.0;
  wristScale = 1.0;
}

void ArmRetargetingManager::CalibResult::load(const mc_rtc::Configuration & mcRtcConfig)
{
  isInitialized = true;

  humanTransFromBaseToShoulder = mcRtcConfig("humanTransFromBaseToShoulder");
  robotTransFromBaseToShoulder = mcRtcConfig("robotTransFromBaseToShoulder");

  elbowRotTransFromHumanToRobot = mcRtcConfig("elbowRotTransFromHumanToRobot");
  wristRotTransFromHumanToRobot = mcRtcConfig("wristRotTransFromHumanToRobot");

  elbowScale = mcRtcConfig("elbowScale");
  wristScale = mcRtcConfig("wristScale");
}

std::string ArmRetargetingManager::CalibResult::dump() const
{
  mc_rtc::Configuration mcRtcConfig;

  auto calibConfig = mcRtcConfig.add("calibResultConfig");

  calibConfig.add("humanTransFromBaseToShoulder", humanTransFromBaseToShoulder);
  calibConfig.add("robotTransFromBaseToShoulder", robotTransFromBaseToShoulder);

  calibConfig.add("elbowRotTransFromHumanToRobot", elbowRotTransFromHumanToRobot);
  calibConfig.add("wristRotTransFromHumanToRobot", wristRotTransFromHumanToRobot);

  calibConfig.add("elbowScale", elbowScale);
  calibConfig.add("wristScale", wristScale);

  return mcRtcConfig.dump(true, true);
}

ArmRetargetingManager::ArmRetargetingManager(HumanRetargetingController * ctlPtr,
                                             const mc_rtc::Configuration & mcRtcConfig)
: ctlPtr_(ctlPtr)
{
  config_.load(mcRtcConfig);
}

void ArmRetargetingManager::reset()
{
  humanElbowPoseManager_ = std::make_shared<RosPoseManager>(ctlPtr_, config_.humanElbowPoseTopicName);
  humanWristPoseManager_ = std::make_shared<RosPoseManager>(ctlPtr_, config_.humanWristPoseTopicName);

  humanShoulderPose_ = std::nullopt;
  humanElbowPose_ = std::nullopt;
  humanWristPose_ = std::nullopt;

  robotShoulderPose_ = std::nullopt;
  robotElbowPose_ = std::nullopt;
  robotWristPose_ = std::nullopt;

  stiffnessRatioFunc_ = nullptr;

  humanCalibSource_.clear();
  robotCalibSource_.clear();

  if(!config_.calibResultConfig.empty())
  {
    calibResult_.load(config_.calibResultConfig);
  }
}

void ArmRetargetingManager::update()
{
  // Calculate robot pose
  if(calibResult_.isInitialized)
  {
    humanShoulderPose_ = std::nullopt;
    humanElbowPose_ = std::nullopt;
    humanWristPose_ = std::nullopt;

    robotShoulderPose_ = calibResult_.robotTransFromBaseToShoulder
                         * ctl().robot().frame(ctl().retargetingManagerSet_->config().robotBaseLinkName).position();
    robotElbowPose_ = std::nullopt;
    robotWristPose_ = std::nullopt;

    if(humanWaistPoseManager()->isValid())
    {
      auto scalePose = [&](const sva::PTransformd & origPose, double scale) -> sva::PTransformd {
        sva::PTransformd newPose = origPose;
        newPose.translation() *= scale;
        return newPose;
      };

      const auto & humanWaistPoseFromOrigin = ctl().retargetingManagerSet_->config_.humanWaistPoseFromOrigin;
      humanShoulderPose_ = calibResult_.humanTransFromBaseToShoulder * humanWaistPoseFromOrigin;

      if(humanElbowPoseManager_->isValid())
      {
        humanElbowPose_ =
            humanElbowPoseManager_->pose() * humanWaistPoseManager()->pose().inv() * humanWaistPoseFromOrigin;
        robotElbowPose_ =
            calibResult_.elbowRotTransFromHumanToRobot
            * scalePose(humanElbowPose_.value() * humanShoulderPose_.value().inv(), calibResult_.elbowScale)
            * robotShoulderPose_.value();
      }
      if(humanWristPoseManager_->isValid())
      {
        humanWristPose_ =
            humanWristPoseManager_->pose() * humanWaistPoseManager()->pose().inv() * humanWaistPoseFromOrigin;
        robotWristPose_ =
            calibResult_.wristRotTransFromHumanToRobot
            * scalePose(humanWristPose_.value() * humanShoulderPose_.value().inv(), calibResult_.wristScale)
            * robotShoulderPose_.value();
      }
    }
  }

  // Update task target
  if(ctl().retargetingManagerSet_->isEnabled_)
  {
    updateTaskTarget(elbowTask(), robotElbowPose_.value());
    updateTaskTarget(wristTask(), robotWristPose_.value());
  }

  // Interpolate task stiffness
  if(stiffnessRatioFunc_)
  {
    if(ctl().t() < stiffnessRatioFunc_->endTime())
    {
      double stiffnessRatio = (*stiffnessRatioFunc_)(ctl().t());
      for(const auto & task : {elbowTask(), wristTask()})
      {
        task->stiffness(stiffnessRatio * config_.stiffness);
      }
    }
    else
    {
      for(const auto & task : {elbowTask(), wristTask()})
      {
        task->stiffness(Eigen::VectorXd(config_.stiffness));
      }
      stiffnessRatioFunc_.reset();
    }
  }
}

void ArmRetargetingManager::stop()
{
  for(const auto & task : {elbowTask(), wristTask()})
  {
    ctl().solver().removeTask(task);
  }

  removeFromGUI(*ctl().gui());
  removeFromLogger(ctl().logger());
}

void ArmRetargetingManager::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  const std::vector<std::string> & calibCategory = {ctl().name(), ctl().retargetingManagerSet_->config_.name,
                                                    std::to_string(config_.armSide), "Calib"};

  gui.addElement(calibCategory,
                 mc_rtc::gui::Label("isInitialized", [this]() { return calibResult_.isInitialized ? "Yes" : "No"; }),
                 mc_rtc::gui::Button("reset", [this]() { calibResult_.reset(); }),
                 mc_rtc::gui::Button("update", [this]() { updateCalib(); }));
  gui.addElement(calibCategory, mc_rtc::gui::ElementsStacking::Horizontal,
                 mc_rtc::gui::Button("setHuman-X", [this]() { setHumanCalibSource("X"); }),
                 mc_rtc::gui::Button("setHuman-Y", [this]() { setHumanCalibSource("Y"); }),
                 mc_rtc::gui::Button("setHuman-Z", [this]() { setHumanCalibSource("Z"); }));
  gui.addElement(calibCategory, mc_rtc::gui::ElementsStacking::Horizontal,
                 mc_rtc::gui::Button("setRobot-X", [this]() { setRobotCalibSource("X"); }),
                 mc_rtc::gui::Button("setRobot-Y", [this]() { setRobotCalibSource("Y"); }),
                 mc_rtc::gui::Button("setRobot-Z", [this]() { setRobotCalibSource("Z"); }));
}

void ArmRetargetingManager::removeFromGUI(mc_rtc::gui::StateBuilder & gui)
{
  gui.removeCategory({ctl().name(), ctl().retargetingManagerSet_->config_.name, std::to_string(config_.armSide)});
}

void ArmRetargetingManager::addToLogger(mc_rtc::Logger & logger)
{
  const auto & name = ctl().retargetingManagerSet_->config_.name + "_" + std::to_string(config_.armSide);

  logger.addLogEntry(name + "_humanElbowValid", this, [this]() { return humanElbowPoseManager_->isValid(); });
  logger.addLogEntry(name + "_humanElbowPose", this, [this]() {
    return humanElbowPoseManager_->isValid() ? humanElbowPoseManager_->pose() : sva::PTransformd::Identity();
  });
  logger.addLogEntry(name + "_humanWristValid", this, [this]() { return humanWristPoseManager_->isValid(); });
  logger.addLogEntry(name + "_humanWristPose", this, [this]() {
    return humanWristPoseManager_->isValid() ? humanWristPoseManager_->pose() : sva::PTransformd::Identity();
  });
}

void ArmRetargetingManager::removeFromLogger(mc_rtc::Logger & logger)
{
  logger.removeLogEntries(this);
}

void ArmRetargetingManager::enable()
{
  for(const auto & task : {elbowTask(), wristTask()})
  {
    task->reset();
    ctl().solver().addTask(task);
    task->stiffness(0.0);
  }

  constexpr double stiffnessInterpDuration = 2.0; // [sec]
  stiffnessRatioFunc_ = std::make_shared<TrajColl::CubicInterpolator<double>>(
      std::map<double, double>{{ctl().t(), 0.0}, {ctl().t() + stiffnessInterpDuration, 1.0}});
}

void ArmRetargetingManager::disable()
{
  for(const auto & task : {elbowTask(), wristTask()})
  {
    ctl().solver().removeTask(task);
  }

  stiffnessRatioFunc_ = nullptr;
}

const std::shared_ptr<RosPoseManager> & ArmRetargetingManager::humanWaistPoseManager() const
{
  return ctl().retargetingManagerSet_->humanWaistPoseManager_;
}

const std::shared_ptr<mc_tasks::TransformTask> & ArmRetargetingManager::elbowTask() const
{
  return ctl().retargetingTasks_.at(config_.elbowTaskName);
}

const std::shared_ptr<mc_tasks::TransformTask> & ArmRetargetingManager::wristTask() const
{
  return ctl().retargetingTasks_.at(config_.wristTaskName);
}

void ArmRetargetingManager::updateTaskTarget(const std::shared_ptr<mc_tasks::TransformTask> & task,
                                             const sva::PTransformd & pose)
{
  if(const auto & impTask = std::dynamic_pointer_cast<mc_tasks::force::ImpedanceTask>(task))
  {
    impTask->targetPose(pose);
    impTask->targetVel(sva::MotionVecd::Zero());
    impTask->targetAccel(sva::MotionVecd::Zero());
  }
  else
  {
    task->target(pose);
    task->targetVel(sva::MotionVecd::Zero());
  }
}

void ArmRetargetingManager::setHumanCalibSource(const std::string & axis)
{
  if(!(humanWaistPoseManager()->isValid() && humanElbowPoseManager_->isValid() && humanWristPoseManager_->isValid()))
  {
    mc_rtc::log::error("[ArmRetargetingManager({})] Human pose is invalid. Waist: {}, Elbow: {}, Wrist: {}",
                       std::to_string(config_.armSide), humanWaistPoseManager()->isValid(),
                       humanElbowPoseManager_->isValid(), humanWristPoseManager_->isValid());
    return;
  }

  const auto & basePose = humanWaistPoseManager()->pose();
  const auto & elbowPose = humanElbowPoseManager_->pose();
  const auto & wristPose = humanWristPoseManager_->pose();
  humanCalibSource_.emplace(axis,
                            std::array<sva::PTransformd, 2>{elbowPose * basePose.inv(), wristPose * basePose.inv()});
}

void ArmRetargetingManager::setRobotCalibSource(const std::string & axis)
{
  ctl().retargetingManagerSet_->makeCalibRobot();

  auto & calibRobot = ctl().retargetingManagerSet_->calibRobots_->robot();
  for(const auto & [jointName, jointPos] : config_.robotCalibPostures.at(axis))
  {
    calibRobot.q()[calibRobot.jointIndexByName(jointName)][0] = jointPos;
  }
  calibRobot.forwardKinematics();

  const auto & basePose = calibRobot.frame(ctl().retargetingManagerSet_->config().robotBaseLinkName).position();
  const auto & elbowPose = calibRobot.frame(elbowTask()->frame().name()).position();
  const auto & wristPose = calibRobot.frame(wristTask()->frame().name()).position();
  robotCalibSource_.emplace(axis,
                            std::array<sva::PTransformd, 2>{elbowPose * basePose.inv(), wristPose * basePose.inv()});
}

void ArmRetargetingManager::updateCalib()
{
  ctl().retargetingManagerSet_->clearCalibRobot();

  if(!(humanCalibSource_.count("X") && humanCalibSource_.count("Y") && humanCalibSource_.count("Z")
       && robotCalibSource_.count("X") && robotCalibSource_.count("Y") && robotCalibSource_.count("Z")))
  {
    mc_rtc::log::error("[ArmRetargetingManager({})] Calibration source is missing. Human: (X: {}, Y: {}, Z: {}), "
                       "Robot: (X: {}, Y: {}, Z: {})",
                       std::to_string(config_.armSide), humanCalibSource_.count("X"), humanCalibSource_.count("Y"),
                       humanCalibSource_.count("Z"), robotCalibSource_.count("X"), robotCalibSource_.count("Y"),
                       robotCalibSource_.count("Z"));
    return;
  }

  calibResult_.isInitialized = true;

  calibResult_.humanTransFromBaseToShoulder = calcShoulderPoseForCalib(humanCalibSource_);
  calibResult_.robotTransFromBaseToShoulder = calcShoulderPoseForCalib(robotCalibSource_);

  calibResult_.elbowRotTransFromHumanToRobot = robotCalibSource_.at("X")[0] * humanCalibSource_.at("X")[0].inv();
  calibResult_.elbowRotTransFromHumanToRobot.translation().setZero();
  calibResult_.wristRotTransFromHumanToRobot = robotCalibSource_.at("X")[1] * humanCalibSource_.at("X")[1].inv();
  calibResult_.wristRotTransFromHumanToRobot.translation().setZero();

  const auto & humanElbowWristLengths =
      calcLimbLengthsForCalib(humanCalibSource_, calibResult_.humanTransFromBaseToShoulder);
  const auto & robotElbowWristLengths =
      calcLimbLengthsForCalib(robotCalibSource_, calibResult_.robotTransFromBaseToShoulder);
  calibResult_.elbowScale = robotElbowWristLengths[0] / humanElbowWristLengths[0];
  calibResult_.wristScale = robotElbowWristLengths[1] / humanElbowWristLengths[1];

  mc_rtc::log::success("[ArmRetargetingManager({})] Calibration result:\n{}", std::to_string(config_.armSide),
                       calibResult_.dump());
}

sva::PTransformd ArmRetargetingManager::calcShoulderPoseForCalib(const CalibSource & calibSource) const
{
  sva::PTransformd pose;

  std::vector<std::string> axes = {"X", "Y", "Z"};
  Eigen::Matrix3d posMat, dirMat;
  for(size_t i = 0; i < axes.size(); i++)
  {
    posMat.col(i) = calibSource.at(axes[i])[0].translation();
    dirMat.col(i) = (calibSource.at(axes[i])[1].translation() - calibSource.at(axes[i])[0].translation()).normalized();

    if(config_.armSide == ArmSide::Right && axes[i] == "Y")
    {
      posMat.col(i).y() *= -1.0;
      dirMat.col(i).y() *= -1.0;
    }
  }

  {
    Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
    Eigen::Vector3d b = Eigen::Vector3d::Zero();

    auto addContribution = [&](const Eigen::Vector3d & P, const Eigen::Vector3d & d) {
      Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
      A += I - d * d.transpose();
      b += (I - d * d.transpose()) * P;
    };

    for(size_t i = 0; i < axes.size(); i++)
    {
      addContribution(posMat.col(i), dirMat.col(i));
    }

    pose.translation() = A.ldlt().solve(b);
  }

  {
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(dirMat, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    Eigen::Matrix3d rotation = U * V.transpose();

    pose.rotation() = rotation.transpose();
  }

  return pose;
}

std::array<double, 2> ArmRetargetingManager::calcLimbLengthsForCalib(const CalibSource & calibSource,
                                                                     const sva::PTransformd & shoulderPose) const
{
  std::array<double, 2> elbowWristLengths = {0.0, 0.0};

  std::vector<std::string> axes = {"X", "Y", "Z"};
  for(size_t i = 0; i < axes.size(); i++)
  {
    for(int j = 0; j < 2; j++)
    {
      elbowWristLengths[j] += (calibSource.at(axes[i])[j].translation() - shoulderPose.translation()).norm();
    }
  }

  for(int j = 0; j < 2; j++)
  {
    elbowWristLengths[j] /= static_cast<double>(axes.size());
  }

  return elbowWristLengths;
}
