#pragma once

#include <optional>

#include <HumanRetargetingController/RetargetingManager.h>

#include <geometry_msgs/PoseStamped.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <HumanRetargetingController/RetargetingPhase.h>

namespace HRC
{
/** \brief Set of RetargetingManager. */
class RetargetingManagerSet : public std::unordered_map<std::string, std::shared_ptr<RetargetingManager>>
{
  // Allow access to protected members in RetargetingManagerSet from RetargetingManager
  friend class RetargetingManager;

  /** \brief Configuration. */
  struct Configuration
  {
    //! Name
    std::string name = "RetargetingManagerSet";

    //! Base frame name
    std::string baseFrame = "base_frame";

    //! Topic name of base pose
    std::string basePoseTopicName = "/hrc/poses/base";

    //! Expiration duration of pose [s]
    double poseExpirationDuration = 3.0;

    //! Distance threshold from base to target body part [m]
    double targetDistThre = 2.0;

    //! Velocity (difference from last pose) threshold of target body part
    double targetVelThre = 0.5;

    //! Point marker size
    double pointMarkerSize = 0.15;

    //! Base marker size (width, height) [m]
    Eigen::Vector2d baseMarkerSize = Eigen::Vector2d(0.4, 0.5);

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

  /** \brief Update the validity. */
  void updateValidity();

  /** \brief Update the phase. */
  void updatePhase();

  /** \brief Update GUI. */
  void updateGUI();

  /** \brief Get the marker color of retargeting phase. */
  mc_rtc::gui::Color getRetargetingPhaseColor() const;

  /** \brief ROS callback of base pose topic. */
  void basePoseCallback(const geometry_msgs::PoseStamped::ConstPtr & poseStMsg);

protected:
  //! Configuration
  Configuration config_;

  //! Pointer to controller
  HumanRetargetingController * ctlPtr_ = nullptr;

  //! Whether it is ready for retargeting
  bool isReady_ = false;

  //! Retargeting phase
  RetargetingPhase retargetingPhase_ = RetargetingPhase::Disabled;

  //! Human base pose represented in world frame
  std::optional<sva::PTransformd> humanBasePose_ = std::nullopt;

  //! Robot base pose represented in world frame
  sva::PTransformd robotBasePose_ = sva::PTransformd::Identity();

  //! Time when the latest base pose was obtained
  double basePoseLatestTime_ = -1;

  //! ROS node handle
  std::shared_ptr<ros::NodeHandle> nh_;

  //! ROS callback queue
  ros::CallbackQueue callbackQueue_;

  //! ROS subscriber of base pose
  ros::Subscriber basePoseSub_;
};
} // namespace HRC
