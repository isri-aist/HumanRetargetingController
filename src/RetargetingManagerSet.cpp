#include <mc_rtc/gui/Arrow.h>
#include <mc_rtc/gui/Button.h>
#include <mc_rtc/gui/Cylinder.h>
#include <mc_rtc/gui/Ellipsoid.h>
#include <mc_rtc/gui/Label.h>
#include <mc_rtc/gui/Point3D.h>

#include <HumanRetargetingController/ArmRetargetingManager.h>
#include <HumanRetargetingController/HumanRetargetingController.h>
#include <HumanRetargetingController/RetargetingManagerSet.h>
#include <HumanRetargetingController/RosPoseManager.h>

#include <sensor_msgs/Joy.h>

using namespace HRC;

void RetargetingManagerSet::Configuration::load(const mc_rtc::Configuration & mcRtcConfig)
{
  mcRtcConfig("name", name);

  humanWaistPoseTopicName = static_cast<std::string>(mcRtcConfig("humanWaistPoseTopicName"));

  robotBaseLinkName = static_cast<std::string>(mcRtcConfig("robotBaseLinkName"));

  mcRtcConfig("syncJoints", syncJoints);

  mcRtcConfig("pointMarkerSize", pointMarkerSize);
  mcRtcConfig("phaseMarkerPoseOffset", phaseMarkerPoseOffset);
}

RetargetingManagerSet::RetargetingManagerSet(HumanRetargetingController * ctlPtr,
                                             const mc_rtc::Configuration & mcRtcConfig)
: ctlPtr_(ctlPtr)
{
  config_.load(mcRtcConfig);

  for(auto [armSide, retargetingManagerConfig] :
      static_cast<std::map<std::string, mc_rtc::Configuration>>(mcRtcConfig("ArmRetargetingManagerList")))
  {
    retargetingManagerConfig.add("armSide", armSide);
    this->emplace(strToArmSide(armSide), std::make_shared<ArmRetargetingManager>(ctlPtr, retargetingManagerConfig));
  }
}

void RetargetingManagerSet::reset()
{
  // Setup ROS
  if(nh_)
  {
    mc_rtc::log::error("[RetargetingManagerSet] ROS node handle is already instantiated.");
    nh_.reset();
  }
  nh_ = std::make_shared<ros::NodeHandle>();
  // Use a dedicated queue so as not to call callbacks of other modules
  nh_->setCallbackQueue(&callbackQueue_);

  // Note: ROS node handle must be instantiated first
  humanWaistPoseManager_ = std::make_shared<RosPoseManager>(ctlPtr_, config_.humanWaistPoseTopicName);

  isReady_ = false;
  isEnabled_ = false;

  for(const auto & armManagerKV : *this)
  {
    armManagerKV.second->reset();
  }
}

void RetargetingManagerSet::update()
{
  callbackQueue_.callAvailable(ros::WallDuration());

  updateReadiness();
  updateEnablement();

  for(const auto & armManagerKV : *this)
  {
    armManagerKV.second->update();
  }

  updateGUI();
}

void RetargetingManagerSet::stop()
{
  removeFromGUI(*ctl().gui());
  removeFromLogger(ctl().logger());

  for(const auto & armManagerKV : *this)
  {
    armManagerKV.second->stop();
  }
}

void RetargetingManagerSet::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  gui.addElement({ctl().name(), config_.name, "Status"},
                 mc_rtc::gui::Label("isReady", [this]() { return isReady_ ? "Yes" : "No"; }),
                 mc_rtc::gui::Label("isEnabled", [this]() { return isEnabled_ ? "Yes" : "No"; }));

  for(const auto & armManagerKV : *this)
  {
    armManagerKV.second->addToGUI(gui);
  }
}

void RetargetingManagerSet::removeFromGUI(mc_rtc::gui::StateBuilder & gui)
{
  gui.removeCategory({ctl().name(), config_.name});

  // GUI of each RetargetingManager is not removed here (removed via stop method)
}

void RetargetingManagerSet::addToLogger(mc_rtc::Logger & logger)
{
  MC_RTC_LOG_HELPER(config_.name + "_isReady", isReady_);
  MC_RTC_LOG_HELPER(config_.name + "_isEnabled", isEnabled_);

  logger.addLogEntry(config_.name + "_humanWaistValid", this, [this]() { return humanWaistPoseManager_->isValid(); });
  logger.addLogEntry(config_.name + "_humanWaistPose", this, [this]() {
    return humanWaistPoseManager_->isValid() ? humanWaistPoseManager_->pose() : sva::PTransformd::Identity();
  });

  for(const auto & armManagerKV : *this)
  {
    armManagerKV.second->addToLogger(logger);
  }
}

void RetargetingManagerSet::removeFromLogger(mc_rtc::Logger & logger)
{
  logger.removeLogEntries(this);

  // Log of each RetargetingManager is not removed here (removed via stop method)
}

void RetargetingManagerSet::enable()
{
  if(!isReady_)
  {
    mc_rtc::log::error("[RetargetingManagerSet] Retargeting is not ready.");
    return;
  }

  if(isEnabled_)
  {
    mc_rtc::log::error("[RetargetingManagerSet] Retargeting is already enabled.");
    return;
  }

  isEnabled_ = true;
  mc_rtc::log::success("[RetargetingManagerSet] Enable retargeting.");

  for(const auto & armManagerKV : *this)
  {
    armManagerKV.second->enable();
  }
}

void RetargetingManagerSet::disable()
{
  if(!isEnabled_)
  {
    mc_rtc::log::error("[RetargetingManagerSet] Retargeting is already disabled.");
    return;
  }

  isEnabled_ = false;
  mc_rtc::log::success("[RetargetingManagerSet] Disable retargeting.");

  for(const auto & armManagerKV : *this)
  {
    armManagerKV.second->disable();
  }

  // Update target position of joints
  {
    std::map<std::string, std::vector<double>> targetPosture;
    const auto & robotQ = ctl().robot().q();
    for(const auto & jointName : config_.syncJoints)
    {
      targetPosture.emplace(jointName, robotQ[ctl().robot().jointIndexByName(jointName)]);
    }

    auto postureTask = ctl().getPostureTask(ctl().robot().name());
    postureTask->target(targetPosture);
  }
}

void RetargetingManagerSet::updateReadiness()
{
  // Check readiness
  isReady_ = true;
  std::string invalidReasonStr = "";

  if(!humanWaistPoseManager_->isValid())
  {
    isReady_ = false;
    invalidReasonStr += "[Waist] " + humanWaistPoseManager_->invalidReasonStr_;
  }

  for(const auto & armManagerKV : *this)
  {
    if(!armManagerKV.second->calibResult_.isInitialized)
    {
      isReady_ = false;
      invalidReasonStr += "[" + std::to_string(armManagerKV.first) + "Arm] calib-uninitialized; ";
    }

    std::map<std::string, std::shared_ptr<RosPoseManager>> poseManagerMap = {
        {"Elbow", armManagerKV.second->humanElbowPoseManager_}, {"Wrist", armManagerKV.second->humanWristPoseManager_}};

    for(const auto & [poseName, poseManager] : poseManagerMap)
    {
      if(humanWaistPoseManager_->isValid())
      {
        constexpr double distThre = 2.0; // [m]
        poseManager->setDistValidityCheck(humanWaistPoseManager_->pose().translation(), distThre);
      }
      else
      {
        poseManager->clearDistValidityCheck();
      }

      if(!poseManager->isValid())
      {
        isReady_ = false;
        invalidReasonStr +=
            "[" + std::to_string(armManagerKV.first) + poseName + "] " + humanWaistPoseManager_->invalidReasonStr_;
      }
    }
  }

  // Disable retargeting if not ready
  if(!isReady_ && isEnabled_)
  {
    mc_rtc::log::error("Disable retargeting because it is not ready: {}", invalidReasonStr);
    disable();
  }
}

void RetargetingManagerSet::updateEnablement()
{
  bool enableFlag = false;
  bool disableFlag = false;

  if(ctl().datastore().has("HRC::ViveRos::LeftHandJoyMsg"))
  {
    const sensor_msgs::Joy & leftHandJoyMsg = ctl().datastore().get<sensor_msgs::Joy>("HRC::ViveRos::LeftHandJoyMsg");

    if(leftHandJoyMsg.buttons[0])
    {
      disableFlag = true;
    }

    ctl().datastore().remove("HRC::ViveRos::LeftHandJoyMsg");
  }
  if(ctl().datastore().has("HRC::ViveRos::RightHandJoyMsg"))
  {
    const sensor_msgs::Joy & rightHandJoyMsg = ctl().datastore().get<sensor_msgs::Joy>("HRC::ViveRos::RightHandJoyMsg");

    if(rightHandJoyMsg.buttons[0])
    {
      enableFlag = true;
    }

    ctl().datastore().remove("HRC::ViveRos::RightHandJoyMsg");
  }

  if(disableFlag)
  {
    disable();
  }
  else if(enableFlag)
  {
    enable();
  }
}

void RetargetingManagerSet::updateGUI()
{
  // Add buttons
  ctl().gui()->removeElement({ctl().name(), config_.name}, "EnableRetargeting");
  ctl().gui()->removeElement({ctl().name(), config_.name}, "DisableRetargeting");
  if(isReady_ && !isEnabled_)
  {
    ctl().gui()->addElement({ctl().name(), config_.name},
                            mc_rtc::gui::Button("EnableRetargeting", [this]() { enable(); }));
  }
  if(isEnabled_)
  {
    ctl().gui()->addElement({ctl().name(), config_.name},
                            mc_rtc::gui::Button("DisableRetargeting", [this]() { disable(); }));
  }

  // Add pose markers
  auto getRobotPose = [](const std::shared_ptr<ArmRetargetingManager> & _armManager,
                         const std::string & _poseName) -> std::optional<sva::PTransformd> {
    if(_poseName == "Shoulder")
    {
      return _armManager->robotShoulderPose_;
    }
    else if(_poseName == "Elbow")
    {
      return _armManager->robotElbowPose_;
    }
    else if(_poseName == "Wrist")
    {
      return _armManager->robotWristPose_;
    }
    else
    {
      mc_rtc::log::error_and_throw("[RetargetingManagerSet] Invalid pose name: {}", _poseName);
    }
  };
  mc_rtc::gui::Color pointColor = mc_rtc::gui::Color(0.0, 1.0, 0.0, 0.5);
  mc_rtc::gui::Color arrowColor = mc_rtc::gui::Color(0.2, 0.4, 0.2, 0.6);

  ctl().gui()->removeCategory({ctl().name(), config_.name, "Marker"});

  for(const auto & [armSide, armManager] : *this)
  {
    for(const auto & poseName : {"Shoulder", "Elbow", "Wrist"})
    {
      if(getRobotPose(armManager, poseName).has_value())
      {
        ctl().gui()->addElement(
            {ctl().name(), config_.name, "Marker"},
            mc_rtc::gui::Point3D(std::to_string(armSide) + poseName,
                                 mc_rtc::gui::PointConfig(pointColor, config_.pointMarkerSize),
                                 [=]() { return getRobotPose(armManager, poseName).value().translation(); }));
      }
    }

    for(const auto & [poseName1, poseName2] :
        std::vector<std::pair<std::string, std::string>>{{"Shoulder", "Elbow"}, {"Elbow", "Wrist"}})
    {
      if(getRobotPose(armManager, poseName1).has_value() && getRobotPose(armManager, poseName2).has_value())
      {
        ctl().gui()->addElement({ctl().name(), config_.name, "Marker"},
                                mc_rtc::gui::Arrow(
                                    std::to_string(armSide) + poseName1 + "To" + poseName2, arrowColor,
                                    [=]() { return getRobotPose(armManager, poseName1).value().translation(); },
                                    [=]() { return getRobotPose(armManager, poseName2).value().translation(); }));
      }
    }
  }

  // Add phase marker
  auto getPhaseColor = [this]() -> mc_rtc::gui::Color {
    return isEnabled_ ? mc_rtc::gui::Color(1.0, 0.0, 1.0, 0.5) : mc_rtc::gui::Color(0.0, 1.0, 1.0, 0.5);
  };

  if(isReady_)
  {
    ctl().gui()->addElement({ctl().name(), config_.name, "Marker"},
                            mc_rtc::gui::Ellipsoid(
                                "Phase", {0.2, 0.2, 0.15},
                                [this]() { return config_.phaseMarkerPoseOffset * ctl().robot().posW(); },
                                [=]() { return getPhaseColor(); }));
  }
  else
  {
    ctl().gui()->addElement({ctl().name(), config_.name, "Marker"},
                            mc_rtc::gui::Cylinder(
                                "Phase", {0.1, 0.01},
                                [this]() { return config_.phaseMarkerPoseOffset * ctl().robot().posW(); },
                                [=]() { return getPhaseColor(); }));
  }
}
