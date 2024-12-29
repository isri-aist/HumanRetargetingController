#include <iomanip>
#include <sstream>

#include <HumanRetargetingController/HumanRetargetingController.h>
#include <HumanRetargetingController/RetargetingManagerSet.h>
#include <HumanRetargetingController/RosPoseManager.h>

using namespace HRC;

RosPoseManager::RosPoseManager(HumanRetargetingController * ctlPtr, const std::string & topicName) : ctlPtr_(ctlPtr)
{
  // Setup ROS
  const auto & nh = ctl().retargetingManagerSet_->nh_;
  poseSub_ = nh->subscribe<geometry_msgs::PoseStamped>(topicName, 1, &RosPoseManager::poseCallback, this);
}

bool RosPoseManager::isValid()
{
  bool isValid = true;
  invalidReasonStr_.clear();

  if(latestTime_ < 0 || prevTime_ < 0)
  {
    isValid = false;

    invalidReasonStr_ += "Uninitialized-time; ";
  }

  if(latestTime_ <= prevTime_)
  {
    isValid = false;

    invalidReasonStr_ += "Inconsistent-prev-time; ";
  }

  constexpr double durationThre = 3.0; // [s]
  double duration = ctl().t() - latestTime_;
  if(duration > durationThre)
  {
    isValid = false;

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2) << duration;
    invalidReasonStr_ += "Too-old(" + oss.str() + "); ";
  }

  constexpr double velThre = 5.0; // [m/s]
  double vel = (latestPose_.translation() - prevPose_.translation()).norm() / (latestTime_ - prevTime_);
  if(vel > velThre)
  {
    isValid = false;

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2) << vel;
    invalidReasonStr_ += "Too-fast(" + oss.str() + "); ";
  }

  if(distThre_ > 0)
  {
    double dist = (latestPose_.translation() - distOrigin_).norm();
    if(dist > distThre_)
    {
      isValid = false;

      std::ostringstream oss;
      oss << std::fixed << std::setprecision(2) << dist;
      invalidReasonStr_ += "Too-far(" + oss.str() + "); ";
    }
  }

  return isValid;
}

void RosPoseManager::setDistValidityCheck(const Eigen::Vector3d & distOrigin, double distThre)
{
  distOrigin_ = distOrigin;
  distThre_ = distThre;
}

void RosPoseManager::clearDistValidityCheck()
{
  distOrigin_.setZero();
  distThre_ = -1.0;
}

void RosPoseManager::poseCallback(const geometry_msgs::PoseStamped::ConstPtr & poseStMsg)
{
  prevPose_ = latestPose_;
  prevTime_ = latestTime_;

  const auto & oriMsg = poseStMsg->pose.orientation;
  const auto & posMsg = poseStMsg->pose.position;
  latestPose_ = sva::PTransformd(
      Eigen::Quaterniond(oriMsg.w, oriMsg.x, oriMsg.y, oriMsg.z).normalized().toRotationMatrix().transpose(),
      Eigen::Vector3d(posMsg.x, posMsg.y, posMsg.z));
  latestTime_ = ctl().t();
}
