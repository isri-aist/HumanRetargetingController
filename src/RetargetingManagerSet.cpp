#include <mc_rtc/gui/Arrow.h>
#include <mc_rtc/gui/Button.h>
#include <mc_rtc/gui/Cylinder.h>
#include <mc_rtc/gui/Ellipsoid.h>
#include <mc_rtc/gui/Label.h>
#include <mc_rtc/gui/Point3D.h>

#include <state-observation/tools/rigid-body-kinematics.hpp>

#include <BaselineWalkingController/FootManager.h>

#include <HumanRetargetingController/FootTypes.h>
#include <HumanRetargetingController/HumanRetargetingController.h>
#include <HumanRetargetingController/MathUtils.h>
#include <HumanRetargetingController/RetargetingManagerSet.h>

#include <sensor_msgs/Joy.h>

using namespace HRC;

void RetargetingManagerSet::Configuration::load(const mc_rtc::Configuration & mcRtcConfig)
{
  mcRtcConfig("name", name);
  mcRtcConfig("baseFrame", baseFrame);
  mcRtcConfig("basePoseTopicName", basePoseTopicName);
  mcRtcConfig("poseExpirationDuration", poseExpirationDuration);
  mcRtcConfig("targetDistThre", targetDistThre);
  mcRtcConfig("targetVelThre", targetVelThre);
  mcRtcConfig("pointMarkerSize", pointMarkerSize);
  mcRtcConfig("baseMarkerSize", baseMarkerSize);
  mcRtcConfig("phaseMarkerPoseOffset", phaseMarkerPoseOffset);
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
  isReady_ = false;
  retargetingPhase_ = RetargetingPhase::Disabled;
  humanBasePose_ = std::nullopt;
  robotBasePose_ = sva::PTransformd::Identity();
  basePoseLatestTime_ = -1;

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

  // Pre-update each RetargetingManager
  for(const auto & limbManagerKV : *this)
  {
    limbManagerKV.second->preUpdate();
  }

  // Update robot base pose
  {
    robotBasePose_ = ctl().robot().frame(config_.baseFrame).position();
    // Assume robot base pose is parallel to foot middle pose
    robotBasePose_.rotation() = projGround(sva::interpolate(ctl().footManager_->targetFootPose(Foot::Left),
                                                            ctl().footManager_->targetFootPose(Foot::Right), 0.5))
                                    .rotation();
  }

  // Common update
  updateValidity();
  updatePhase();

  // Update each RetargetingManager
  for(const auto & limbManagerKV : *this)
  {
    limbManagerKV.second->postUpdate();
  }

  // Update GUI marker
  updateGUI();
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

  gui.addElement({ctl().name(), config_.name, "Status"},
                 mc_rtc::gui::Label("retargetingPhase", [this]() { return std::to_string(retargetingPhase_); }),
                 mc_rtc::gui::Label("isReady", [this]() { return isReady_ ? "Yes" : "No"; }));
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

  logger.addLogEntry(config_.name + "_retargetingPhase", this, [this]() { return std::to_string(retargetingPhase_); });
  MC_RTC_LOG_HELPER(config_.name + "_isReady", isReady_);

  logger.addLogEntry(config_.name + "_humanBasePose", this,
                     [this]() { return humanBasePose_.value_or(sva::PTransformd::Identity()); });
  MC_RTC_LOG_HELPER(config_.name + "_robotBasePose", robotBasePose_);
}

void RetargetingManagerSet::removeFromLogger(mc_rtc::Logger & // logger
)
{
  // Log of each RetargetingManager is not removed here (removed via stop method)
}

void RetargetingManagerSet::enable()
{
  retargetingPhase_ = RetargetingPhase::Enabled;
  mc_rtc::log::success("[RetargetingManagerSet] Enable retargeting.");

  for(const auto & limbManagerKV : *this)
  {
    limbManagerKV.second->enable();
  }
}

void RetargetingManagerSet::disable()
{
  retargetingPhase_ = RetargetingPhase::Disabled;
  mc_rtc::log::success("[RetargetingManagerSet] Disable retargeting.");

  for(const auto & limbManagerKV : *this)
  {
    limbManagerKV.second->disable();
  }
}

void RetargetingManagerSet::freeze()
{
  retargetingPhase_ = RetargetingPhase::Frozen;
  mc_rtc::log::success("[RetargetingManagerSet] Freeze retargeting.");

  for(const auto & limbManagerKV : *this)
  {
    limbManagerKV.second->freeze();
  }
}

void RetargetingManagerSet::updateValidity()
{
  // Check base pose validity
  {
    bool poseValid = true;

    if(poseValid && !humanBasePose_.has_value())
    {
      poseValid = false;
    }

    if(poseValid && (basePoseLatestTime_ < ctl().t() - config_.poseExpirationDuration))
    {
      poseValid = false;
    }

    if(!poseValid)
    {
      humanBasePose_ = std::nullopt;
    }
  }

  // Check readiness
  {
    isReady_ = true;

    if(isReady_ && !humanBasePose_.has_value())
    {
      isReady_ = false;
    }

    for(const auto & limbManagerKV : *this)
    {
      if(!isReady_)
      {
        break;
      }
      if(limbManagerKV.second->humanTargetPose_.has_value())
      {
        // Check target pose distance for safety
        double targetDist =
            (limbManagerKV.second->humanTargetPose_.value() * humanBasePose_.value().inv()).translation().norm();
        if(targetDist > config_.targetDistThre)
        {
          isReady_ = false;
          continue;
        }

        // Check target pose velocity for safety
        const auto & targetVel = limbManagerKV.second->humanTargetPoseVel();
        if(targetVel.has_value() && targetVel.value().linear().norm() > config_.targetVelThre)
        {
          isReady_ = false;
          continue;
        }
      }
      else
      {
        isReady_ = false;
      }
    }
  }

  // Freeze retargeting if not ready
  if(retargetingPhase_ == RetargetingPhase::Enabled && !isReady_)
  {
    freeze();
  }
}

void RetargetingManagerSet::updatePhase()
{
  bool advanceFlag = false;
  bool backwardFlag = false;
  bool suspendFlag = false;

  if(ctl().datastore().has("HRC::ViveRos::LeftHandJoyMsg"))
  {
    sensor_msgs::Joy leftHandJoyMsg = ctl().datastore().get<sensor_msgs::Joy>("HRC::ViveRos::LeftHandJoyMsg");
    if(leftHandJoyMsg.buttons[0])
    {
      backwardFlag = true;
    }
    if(leftHandJoyMsg.buttons[1])
    {
      suspendFlag = true;
    }

    ctl().datastore().remove("HRC::ViveRos::LeftHandJoyMsg");
  }
  if(ctl().datastore().has("HRC::ViveRos::RightHandJoyMsg"))
  {
    sensor_msgs::Joy rightHandJoyMsg = ctl().datastore().get<sensor_msgs::Joy>("HRC::ViveRos::RightHandJoyMsg");
    if(rightHandJoyMsg.buttons[0])
    {
      advanceFlag = true;
    }
    if(rightHandJoyMsg.buttons[1])
    {
      suspendFlag = true;
    }

    ctl().datastore().remove("HRC::ViveRos::RightHandJoyMsg");
  }

  if(isReady_ && retargetingPhase_ != RetargetingPhase::Enabled)
  {
    if(advanceFlag)
    {
      enable();
    }
  }
  if(retargetingPhase_ != RetargetingPhase::Disabled)
  {
    if(backwardFlag)
    {
      disable();
    }
  }
  if(retargetingPhase_ == RetargetingPhase::Enabled)
  {
    if(suspendFlag)
    {
      freeze();
    }
  }
}

void RetargetingManagerSet::updateGUI()
{
  // Add buttons
  ctl().gui()->removeElement({ctl().name(), config_.name}, "EnableRetargeting");
  ctl().gui()->removeElement({ctl().name(), config_.name}, "DisableRetargeting");
  ctl().gui()->removeElement({ctl().name(), config_.name}, "FreezeRetargeting");
  if(isReady_ && retargetingPhase_ != RetargetingPhase::Enabled)
  {
    ctl().gui()->addElement({ctl().name(), config_.name},
                            mc_rtc::gui::Button("EnableRetargeting", [this]() { enable(); }));
  }
  if(retargetingPhase_ != RetargetingPhase::Disabled)
  {
    ctl().gui()->addElement({ctl().name(), config_.name},
                            mc_rtc::gui::Button("DisableRetargeting", [this]() { disable(); }));
  }
  if(retargetingPhase_ == RetargetingPhase::Enabled)
  {
    ctl().gui()->addElement({ctl().name(), config_.name},
                            mc_rtc::gui::Button("FreezeRetargeting", [this]() { freeze(); }));
  }

  // Add pose markers
  mc_rtc::gui::Color pointColor = mc_rtc::gui::Color(0.0, 1.0, 0.0, 0.5);
  mc_rtc::gui::Color arrowColor = mc_rtc::gui::Color(0.2, 0.4, 0.2, 0.6);

  ctl().gui()->removeCategory({ctl().name(), config_.name, "Marker"});

  ctl().gui()->addElement({ctl().name(), config_.name, "Marker"},
                          mc_rtc::gui::Point3D("BasePoint",
                                               mc_rtc::gui::PointConfig(pointColor, config_.pointMarkerSize),
                                               [this]() { return robotBasePose_.translation(); }));

  for(const auto & side : std::vector<std::string>{"Left", "Right"})
  {
    ctl().gui()->addElement(
        {ctl().name(), config_.name, "Marker"},
        mc_rtc::gui::Point3D(side + "ShoulderPoint", mc_rtc::gui::PointConfig(pointColor, config_.pointMarkerSize),
                             [this, side]() {
                               double sign = (side == "Left" ? 1.0 : -1.0);
                               return (sva::PTransformd(Eigen::Vector3d(0.0, sign * 0.5 * config_.baseMarkerSize[0],
                                                                        config_.baseMarkerSize[1]))
                                       * robotBasePose_)
                                   .translation();
                             }),
        mc_rtc::gui::Arrow(
            "BaseTo" + side + "ShoulderArrow", arrowColor, [this]() { return robotBasePose_.translation(); },
            [this, side]() {
              double sign = (side == "Left" ? 1.0 : -1.0);
              return (sva::PTransformd(
                          Eigen::Vector3d(0.0, sign * 0.5 * config_.baseMarkerSize[0], config_.baseMarkerSize[1]))
                      * robotBasePose_)
                  .translation();
            }));

    if(this->count(side + "Elbow") > 0 && this->at(side + "Elbow")->robotTargetPose_.has_value())
    {
      ctl().gui()->addElement(
          {ctl().name(), config_.name, "Marker"},
          mc_rtc::gui::Arrow(
              side + "ShoulderToElbowArrow", arrowColor,
              [this, side]() {
                double sign = (side == "Left" ? 1.0 : -1.0);
                return (sva::PTransformd(
                            Eigen::Vector3d(0.0, sign * 0.5 * config_.baseMarkerSize[0], config_.baseMarkerSize[1]))
                        * robotBasePose_)
                    .translation();
              },
              [this, side]() { return this->at(side + "Elbow")->robotTargetPose_.value().translation(); }));

      if(this->count(side + "Hand") > 0 && this->at(side + "Hand")->robotTargetPose_.has_value())
      {
        ctl().gui()->addElement(
            {ctl().name(), config_.name, "Marker"},
            mc_rtc::gui::Arrow(
                side + "ElbowToHandArrow", arrowColor,
                [this, side]() { return this->at(side + "Elbow")->robotTargetPose_.value().translation(); },
                [this, side]() { return this->at(side + "Hand")->robotTargetPose_.value().translation(); }));
      }
    }
  }

  // Add phase marker
  if(isReady_)
  {
    ctl().gui()->addElement({ctl().name(), config_.name, "Marker"},
                            mc_rtc::gui::Ellipsoid(
                                "retargetingPhase", {0.2, 0.2, 0.15},
                                [this]() { return config_.phaseMarkerPoseOffset * robotBasePose_; },
                                [this]() { return getRetargetingPhaseColor(); }));
  }
  else
  {
    ctl().gui()->addElement({ctl().name(), config_.name, "Marker"},
                            mc_rtc::gui::Cylinder(
                                "retargetingPhase", {0.1, 0.01},
                                [this]() { return config_.phaseMarkerPoseOffset * robotBasePose_; },
                                [this]() { return getRetargetingPhaseColor(); }));
  }
}

mc_rtc::gui::Color RetargetingManagerSet::getRetargetingPhaseColor() const
{
  if(retargetingPhase_ == RetargetingPhase::Enabled)
  {
    return mc_rtc::gui::Color(1.0, 0.0, 1.0, 0.5);
  }
  else if(retargetingPhase_ == RetargetingPhase::Disabled)
  {
    return mc_rtc::gui::Color(1.0, 1.0, 0.0, 0.5);
  }
  else // if(retargetingPhase_ == RetargetingPhase::Frozen)
  {
    return mc_rtc::gui::Color(0.0, 1.0, 1.0, 0.5);
  }
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
  // Assume human base pose is always horizontal (only yaw angle can be changed)
  humanBasePose_.value().rotation() = stateObservation::kine::mergeRoll1Pitch1WithYaw2AxisAgnostic(
      Eigen::Matrix3d::Identity(), humanBasePose_.value().rotation());
  basePoseLatestTime_ = ctl().t();
}
