#include <mc_rtc/gui/Arrow.h>
#include <mc_rtc/gui/Label.h>
#include <mc_rtc/gui/Point3D.h>

#include <state-observation/tools/rigid-body-kinematics.hpp>

#include <BaselineWalkingController/FootManager.h>

#include <HumanRetargetingController/FootTypes.h>
#include <HumanRetargetingController/HumanRetargetingController.h>
#include <HumanRetargetingController/MathUtils.h>
#include <HumanRetargetingController/RetargetingManagerSet.h>

using namespace HRC;

void RetargetingManagerSet::Configuration::load(const mc_rtc::Configuration & mcRtcConfig)
{
  mcRtcConfig("name", name);
  mcRtcConfig("baseFrame", baseFrame);
  mcRtcConfig("basePoseTopicName", basePoseTopicName);
  mcRtcConfig("baseMarkerSize", baseMarkerSize);
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
  // Assume robot base pose is parallel to foot middle pose
  robotBasePose_.rotation() = projGround(sva::interpolate(ctl().footManager_->targetFootPose(Foot::Left),
                                                          ctl().footManager_->targetFootPose(Foot::Right), 0.5))
                                  .rotation();

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

  // Update GUI marker
  {
    mc_rtc::gui::Color pointColor = mc_rtc::gui::Color(0.0, 1.0, 0.0, 0.5);
    mc_rtc::gui::Color arrowColor = mc_rtc::gui::Color(0.0, 0.4, 0.2, 0.6);

    ctl().gui()->removeCategory({ctl().name(), config_.name, "Marker"});

    ctl().gui()->addElement({ctl().name(), config_.name, "Marker"},
                            mc_rtc::gui::Point3D("BasePoint", mc_rtc::gui::PointConfig(pointColor, 0.15),
                                                 [this]() { return robotBasePose_.translation(); }));

    for(const auto & side : std::vector<std::string>{"Left", "Right"})
    {
      ctl().gui()->addElement(
          {ctl().name(), config_.name, "Marker"},
          mc_rtc::gui::Point3D(side + "ShoulderPoint", mc_rtc::gui::PointConfig(pointColor, 0.15),
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

  gui.addElement({ctl().name(), config_.name, "Status"},
                 mc_rtc::gui::Label("isTaskEnabled", [this]() { return isTaskEnabled_; }));
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
  // Assume human base pose is always horizontal (only yaw angle can be changed)
  humanBasePose_.value().rotation() = stateObservation::kine::mergeRoll1Pitch1WithYaw2AxisAgnostic(
      Eigen::Matrix3d::Identity(), humanBasePose_.value().rotation());
}
