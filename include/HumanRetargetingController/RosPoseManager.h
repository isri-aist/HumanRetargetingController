#pragma once

#include <SpaceVecAlg/SpaceVecAlg>

#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>

namespace HRC
{
class HumanRetargetingController;

/** \brief ROS pose manager. */
class RosPoseManager
{
public:
  /** \brief Constructor.
      \param ctlPtr pointer to controller
      \param topicName Pose topic name
  */
  RosPoseManager(HumanRetargetingController * ctlPtr, const std::string & topicName);

  /** \brief Get pose. */
  inline sva::PTransformd pose() const
  {
    return latestPose_;
  }

  /** \brief Get whether the pose is valid or not.

      Internally update invalidReasonStr_ so that the reason can be referenced in case of invalid.
   */
  bool isValid();

  /** \brief Set parameters for distance validity check.
      \param distOrigin A point such that the distance between this point and the pose being held is calculated
      \param distThre A threshold such that if it is greater than this threshold, it is determined to be invalid
   */
  void setDistValidityCheck(const Eigen::Vector3d & distOrigin, double distThre);

  /** \brief Clear parameters for distance validity check and disable it. */
  void clearDistValidityCheck();

protected:
  /** \brief ROS callback of pose topic. */
  void poseCallback(const geometry_msgs::msg::PoseStamped & poseStMsg);

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

public:
  //! Latest pose
  sva::PTransformd latestPose_ = sva::PTransformd::Identity();

  //! Latest time
  double latestTime_ = -1.0;

  //! Previous pose
  sva::PTransformd prevPose_ = sva::PTransformd::Identity();

  //! Previous time
  double prevTime_ = -1.0;

  //! String representing the reason for the invalidation
  std::string invalidReasonStr_ = "";

protected:
  //! Pointer to controller
  HumanRetargetingController * ctlPtr_ = nullptr;

  //! Origin to calculate distance
  Eigen::Vector3d distOrigin_ = Eigen::Vector3d::Zero();

  //! Distance threshold (if negative, disables distance validity check)
  double distThre_ = -1.0;

  //! ROS subscriber of pose topic
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr poseSub_;
};
} // namespace HRC
