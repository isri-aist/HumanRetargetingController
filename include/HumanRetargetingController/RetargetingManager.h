#pragma once

#include <optional>

#include <TrajColl/CubicInterpolator.h>

#include <mc_rtc/gui/StateBuilder.h>
#include <mc_rtc/log/Logger.h>

#include <geometry_msgs/PoseStamped.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>

namespace mc_tasks
{
struct TransformTask;

namespace force
{
struct ImpedanceTask;
}
} // namespace mc_tasks

namespace HRC
{
class HumanRetargetingController;

/** \brief Retargeting manager.

    Retargeting manager manages the retargeting task.
*/
class RetargetingManager
{
  // Allow access to config_ in RetargetingManager from RetargetingManagerSet
  friend class RetargetingManagerSet;

public:
  /** \brief Configuration. */
  struct Configuration
  {
    //! Name
    std::string name = "RetargetingManager";

    //! Body part
    std::string bodyPart = "body_part";

    //! Topic name of target pose
    std::string targetPoseTopicName = "";

    //! Retargeting task stiffness
    Eigen::Vector6d stiffness = Eigen::Vector6d::Constant(1000);

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
  RetargetingManager(HumanRetargetingController * ctlPtr, const mc_rtc::Configuration & mcRtcConfig = {});

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

  /** \brief Enable the retargeting task. */
  void enableTask();

  /** \brief Disable the retargeting task. */
  void disableTask();

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

  /** \brief Accessor to the ROS node handle. */
  std::shared_ptr<ros::NodeHandle> nh() const;

  /** \brief Accessor to the human base pose. */
  const std::optional<sva::PTransformd> & humanBasePose() const;

  /** \brief Accessor to the robot base pose. */
  const sva::PTransformd & robotBasePose() const;

  /** \brief Accessor to the retargeting task. */
  const std::shared_ptr<mc_tasks::TransformTask> & retargetingTask() const;

  /** \brief Accessor to the retargeting task as ImpedanceTask.

      Returns nullptr if the retargeting task is not of type ImpedanceTask.
   */
  const std::shared_ptr<mc_tasks::force::ImpedanceTask> retargetingImpTask() const;

  /** \brief ROS callback of target pose topic. */
  void targetPoseCallback(const geometry_msgs::PoseStamped::ConstPtr & poseStMsg);

protected:
  //! Configuration
  Configuration config_;

  //! Pointer to controller
  HumanRetargetingController * ctlPtr_ = nullptr;

  //! Whether the retargeting task is enabled
  bool isTaskEnabled_ = false;

  //! Pose of human target body part represented in world frame
  std::optional<sva::PTransformd> humanTargetPose_ = std::nullopt;

  //! Function to interpolate task stiffness
  std::shared_ptr<TrajColl::CubicInterpolator<double>> stiffnessRatioFunc_;

  //! ROS subscriber of target pose
  ros::Subscriber targetPoseSub_;
};
} // namespace HRC
