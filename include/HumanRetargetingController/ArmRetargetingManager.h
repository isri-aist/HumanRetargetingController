#pragma once

#include <optional>

#include <TrajColl/CubicInterpolator.h>

#include <mc_rtc/gui/StateBuilder.h>
#include <mc_rtc/log/Logger.h>

#include <HumanRetargetingController/ArmSide.h>

namespace mc_tasks
{
struct TransformTask;
} // namespace mc_tasks

namespace HRC
{
class HumanRetargetingController;
class RosPoseManager;

/** \brief Retargeting manager.

    Retargeting manager manages the retargeting task.
*/
class ArmRetargetingManager
{
  // Allow access to protected members
  friend class RetargetingManagerSet;

public:
  /** \brief Configuration. */
  struct Configuration
  {
    //! Arm side
    ArmSide armSide;

    //! Topic name of human elbow pose
    std::string humanElbowPoseTopicName;

    //! Topic name of human wrist pose
    std::string humanWristPoseTopicName;

    //! Name of elbow retargeting task
    std::string elbowTaskName;

    //! Name of wrist retargeting task
    std::string wristTaskName;

    //! Retargeting task stiffness
    Eigen::Vector6d stiffness = Eigen::Vector6d::Constant(10);

    //! Robot postures for calibration
    std::map<std::string, std::map<std::string, double>> robotCalibPostures;

    //! mc_rtc configuration of calibration result
    mc_rtc::Configuration calibResultConfig;

    /** \brief Load mc_rtc configuration.
        \param mcRtcConfig mc_rtc configuration
    */
    void load(const mc_rtc::Configuration & mcRtcConfig);
  };

  /** \brief Calibration result. */
  struct CalibResult
  {
    //! Whether initialized or not
    bool isInitialized;

    //! Transformation from human base (i.e., waist) to shoulder
    sva::PTransformd humanTransFromBaseToShoulder;

    //! Transformation from robot base to shoulder
    sva::PTransformd robotTransFromBaseToShoulder;

    //! Rotational transformation from human elbow to robot elbow
    sva::PTransformd elbowRotTransFromHumanToRobot;

    //! Rotational transformation from human wrist to robot wrist
    sva::PTransformd wristRotTransFromHumanToRobot;

    /** \brief Scale of elbow position relative to shoulder

        Set to a value greater than 1 if the robot's scale is greater than the human's.
    */
    double elbowScale;

    /** \brief Scale of wrist position relative to shoulder

        Set to a value greater than 1 if the robot's scale is greater than the human's.
    */
    double wristScale;

    /** \brief Constructor. */
    CalibResult();

    /** \brief Reset. */
    void reset();

    /** \brief Load mc_rtc configuration.
        \param mcRtcConfig mc_rtc configuration
    */
    void load(const mc_rtc::Configuration & mcRtcConfig);

    /** \brief Get YAML format string. */
    std::string dump() const;
  };

  /** \brief Calibration source.

      The format is {axis, {elbow pose, wrist pose}}.
      The axis is one of "X", "Y", or "Z".
      The elbow pose and wrist pose are expressed relative to the base pose.
   */
  using CalibSource = std::unordered_map<std::string, std::array<sva::PTransformd, 2>>;

public:
  /** \brief Constructor.
      \param ctlPtr pointer to controller
      \param mcRtcConfig mc_rtc configuration
  */
  ArmRetargetingManager(HumanRetargetingController * ctlPtr, const mc_rtc::Configuration & mcRtcConfig = {});

  /** \brief Reset.

      This method should be called once when controller is reset.
  */
  void reset();

  /** \brief Pre-update.

      This method should be called once every control cycle.
  */
  void updatePre();

  /** \brief Post-update.

      This method should be called once every control cycle.
  */
  void updatePost();

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

  /** \brief Accessor to the ROS pose manager for human waist pose. */
  const std::shared_ptr<RosPoseManager> & humanWaistPoseManager() const;

  /** \brief Accessor to the retargeting task of elbow. */
  const std::shared_ptr<mc_tasks::TransformTask> & elbowTask() const;

  /** \brief Accessor to the retargeting task of wrist. */
  const std::shared_ptr<mc_tasks::TransformTask> & wristTask() const;

  /** \brief Set target of retargeting task. */
  void setTaskTarget(const std::shared_ptr<mc_tasks::TransformTask> & task, const sva::PTransformd & pose);

  /** \brief Set human data as calibration source. */
  void setHumanCalibSource(const std::string & axis);

  /** \brief Set robot data as calibration source. */
  void setRobotCalibSource(const std::string & axis);

  /** \brief Update calibration. */
  void updateCalib();

  /** \brief Calculate shoulder pose for calibration. */
  sva::PTransformd calcShoulderPoseForCalib(const CalibSource & calibSource) const;

  /** \brief Calculate lengths of elbow and wrist from shoulder for calibration. */
  std::array<double, 2> calcLimbLengthsForCalib(const CalibSource & calibSource,
                                                const sva::PTransformd & shoulderPose) const;

protected:
  //! Configuration
  Configuration config_;

  //! Pointer to controller
  HumanRetargetingController * ctlPtr_ = nullptr;

  //! ROS pose manager for human elbow pose
  std::shared_ptr<RosPoseManager> humanElbowPoseManager_;

  //! ROS pose manager for human wrist pose
  std::shared_ptr<RosPoseManager> humanWristPoseManager_;

  //! Measured pose of human shoulder (expressed relative to human origin)
  std::optional<sva::PTransformd> humanShoulderPose_ = std::nullopt;

  //! Measured pose of human elbow (expressed relative to human origin)
  std::optional<sva::PTransformd> humanElbowPose_ = std::nullopt;

  //! Measured pose of human wrist (expressed relative to human origin)
  std::optional<sva::PTransformd> humanWristPose_ = std::nullopt;

  //! Target pose of robot shoulder
  std::optional<sva::PTransformd> robotShoulderPose_ = std::nullopt;

  //! Target pose of robot elbow
  std::optional<sva::PTransformd> robotElbowPose_ = std::nullopt;

  //! Target pose of robot wrist
  std::optional<sva::PTransformd> robotWristPose_ = std::nullopt;

  //! Function to interpolate task stiffness
  std::shared_ptr<TrajColl::CubicInterpolator<double>> stiffnessRatioFunc_;

  //! Human data as calibration source
  CalibSource humanCalibSource_;

  //! Robot data as calibration source
  CalibSource robotCalibSource_;

  //! Calibration result
  CalibResult calibResult_;
};
} // namespace HRC
