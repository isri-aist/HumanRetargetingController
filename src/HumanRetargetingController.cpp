#include <mc_tasks/ImpedanceTask.h>
#include <mc_tasks/MetaTaskLoader.h>

#include <BaselineWalkingController/FootManager.h>
#include <BaselineWalkingController/CentroidalManager.h>

#include <HumanRetargetingController/HumanRetargetingController.h>
#include <HumanRetargetingController/RetargetingManagerSet.h>

using namespace HRC;

HumanRetargetingController::HumanRetargetingController(mc_rbdyn::RobotModulePtr rm,
                                         double dt,
                                         const mc_rtc::Configuration & _config,
                                         bool allowEmptyManager)
: BWC::BaselineWalkingController(rm, dt, _config, true)
{
  // Setup tasks
  if(config().has("RetargetingTaskList"))
  {
    for(const auto & retargetingTaskConfig : config()("RetargetingTaskList"))
    {
      std::string bodyPartName = retargetingTaskConfig("bodyPart");
      retargetingTasks_.emplace(bodyPartName,
                             mc_tasks::MetaTaskLoader::load<mc_tasks::force::ImpedanceTask>(solver(), retargetingTaskConfig));
      retargetingTasks_.at(bodyPartName)->name("RetargetingTask_" + bodyPartName);
    }
  }
  else
  {
    mc_rtc::log::warning("[HumanRetargetingController] RetargetingTaskList configuration is missing.");
  }

  // Setup managers
  if(config().has("RetargetingManagerSet"))
  {
    retargetingManagerSet_ = std::make_shared<RetargetingManagerSet>(this, config()("RetargetingManagerSet"));
  }
  else
  {
    mc_rtc::log::warning("[HumanRetargetingController] RetargetingManagerSet configuration is missing.");
  }

  mc_rtc::log::success("[HumanRetargetingController] Constructed.");
}

void HumanRetargetingController::reset(const mc_control::ControllerResetData & resetData)
{
  BaselineWalkingController::reset(resetData);

  mc_rtc::log::success("[HumanRetargetingController] Reset.");
}

bool HumanRetargetingController::run()
{
  t_ += dt();

  if(enableManagerUpdate_)
  {
    // Update managers
    footManager_->update();
    retargetingManagerSet_->update();
    centroidalManager_->update();
  }

  return mc_control::fsm::Controller::run();
}

void HumanRetargetingController::stop()
{
  // Clean up tasks
  for (const auto & [bodyPart, retargetingTask] : retargetingTasks_)
  {
    solver().removeTask(retargetingTask);
  }

  // Clean up managers
  retargetingManagerSet_->stop();

  BaselineWalkingController::stop();
}
