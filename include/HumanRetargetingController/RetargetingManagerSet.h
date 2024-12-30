#pragma once

#include <mc_rtc/gui/StateBuilder.h>
#include <mc_rtc/log/Logger.h>

#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <HumanRetargetingController/ArmSide.h>

namespace HRC
{
class HumanRetargetingController;
class ArmRetargetingManager;
class RosPoseManager;

/** \brief Set of RetargetingManager. */
class RetargetingManagerSet : public std::unordered_map<ArmSide, std::shared_ptr<ArmRetargetingManager>>
{
  // Allow access to protected members
  friend class ArmRetargetingManager;

  /** \brief Configuration. */
  struct Configuration
  {
    //! Name
    std::string name = "RetargetingManagerSet";

    //! Topic name of human waist pose
    std::string humanWaistPoseTopicName;

    //! Name of the robot base link (link at the base of the shoulder)
    std::string robotBaseLinkName;

    //! Joints that update the target position at the end of retargeting
    std::vector<std::string> syncJoints;

    //! Human waist pose from origin
    sva::PTransformd humanWaistPoseFromOrigin = sva::PTransformd::Identity();

    //! Point marker size
    double pointMarkerSize = 0.15;

    //! Pose offset of phase marker
    sva::PTransformd phaseMarkerPoseOffset = sva::PTransformd(Eigen::Vector3d(0.0, 0.0, 1.0));

    /** \brief Load mc_rtc configuration.
        \param mcRtcConfig mc_rtc configuration
    */
    void load(const mc_rtc::Configuration & mcRtcConfig);
  };

public:
  /** \brief Constructor.
      \param ctlPtr pointer to controller
      \param mcRtcConfig mc_rtc configuration
  */
  RetargetingManagerSet(HumanRetargetingController * ctlPtr, const mc_rtc::Configuration & mcRtcConfig = {});

  /** \brief Reset.

      This method should be called once when controller is reset.
  */
  void reset();

  /** \brief Update.

      This method should be called once every control cycle.
  */
  void update();

  /** \brief Stop.

      This method should be called once when stopping the controller.
  */
  void stop();

  /** \brief Const accessor to the configuration. */
  inline const Configuration & config() const noexcept
  {
    return config_;
  }

  /** \brief Add entries to the GUI. */
  void addToGUI(mc_rtc::gui::StateBuilder & gui);

  /** \brief Remove entries from the GUI. */
  void removeFromGUI(mc_rtc::gui::StateBuilder & gui);

  /** \brief Add entries to the logger. */
  void addToLogger(mc_rtc::Logger & logger);

  /** \brief Remove entries from the logger. */
  void removeFromLogger(mc_rtc::Logger & logger);

protected:
  /** \brief Const accessor to the controller. */
  inline const HumanRetargetingController & ctl() const
  {
    return *ctlPtr_;
  }

  /** \brief Accessor to the controller. */
  inline HumanRetargetingController & ctl()
  {
    return *ctlPtr_;
  }

  /** \brief Enable retargeting. */
  void enable();

  /** \brief Disable retargeting. */
  void disable();

  /** \brief Update readiness. */
  void updateReadiness();

  /** \brief Update enablement . */
  void updateEnablement();

  /** \brief Update GUI. */
  void updateGUI();

  /** \brief Make robot for calibration. */
  void makeCalibRobot();

  /** \brief Clear robot for calibration. */
  void clearCalibRobot();

public:
  //! Whether it is ready for retargeting or not
  bool isReady_ = false;

  //! Whether retargeting is enabled or not
  bool isEnabled_ = false;

  //! Robot for calibration
  std::shared_ptr<mc_rbdyn::Robots> calibRobots_;

  //! ROS node handle
  std::shared_ptr<ros::NodeHandle> nh_;

protected:
  //! Configuration
  Configuration config_;

  //! Pointer to controller
  HumanRetargetingController * ctlPtr_ = nullptr;

  //! ROS pose manager for human waist pose
  std::shared_ptr<RosPoseManager> humanWaistPoseManager_;

  //! ROS callback queue
  ros::CallbackQueue callbackQueue_;
};
} // namespace HRC
