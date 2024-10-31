#pragma once

#include <BaselineWalkingController/BaselineWalkingController.h>

namespace mc_tasks
{
namespace force
{
struct ImpedanceTask;
} // namespace force
} // namespace mc_tasks

namespace HRC
{
class RetargetManager;

/** \brief Humanoid loco-manipulation controller. */
struct HumanRetargetingController : public BWC::BaselineWalkingController
{
public:
  /** \brief Constructor.
      \param rm robot module
      \param dt control timestep
      \param _config controller configuration
      \param allowEmptyManager whether to allow the managers to be empty (assuming initialized in the parent class)
   */
  HumanRetargetingController(mc_rbdyn::RobotModulePtr rm,
                      double dt,
                      const mc_rtc::Configuration & config,
                      bool allowEmptyManager = false);

  /** \brief Reset a controller.

      This method is called when starting the controller.
   */
  void reset(const mc_control::ControllerResetData & resetData) override;

  /** \brief Run a controller.

      This method is called every control period.
   */
  bool run() override;

  /** \brief Stop a controller.

      This method is called when stopping the controller.
   */
  void stop() override;

public:
  //! Hand tasks
  std::unordered_map<std::string, std::shared_ptr<mc_tasks::force::ImpedanceTask>> retargetTasks_;

  //! Manipulation manager
  std::shared_ptr<RetargetManager> retargetManager_;
};
} // namespace HRC
