#include <mc_tasks/ImpedanceTask.h>
#include <mc_tasks/MetaTaskLoader.h>

#include <BaselineWalkingController/FootManager.h>
#include <BaselineWalkingController/CentroidalManager.h>

#include <HumanRetargetingController/HumanRetargetingController.h>
#include <HumanRetargetingController/RetargetManager.h>

using namespace HRC;

HumanRetargetingController::HumanRetargetingController(mc_rbdyn::RobotModulePtr rm,
                                         double dt,
                                         const mc_rtc::Configuration & _config,
                                         bool allowEmptyManager)
: BWC::BaselineWalkingController(rm, dt, _config, true)
{
  // Setup tasks
  if(config().has("RetargetTaskList"))
  {
    for(const auto & retargetTaskConfig : config()("RetargetTaskList"))
    {
      std::string bodyPartName = retargetTaskConfig("bodyPart");
      retargetTasks_.emplace(bodyPartName,
                             mc_tasks::MetaTaskLoader::load<mc_tasks::force::ImpedanceTask>(solver(), retargetTaskConfig));
      retargetTasks_.at(bodyPartName)->name("RetargetTask_" + bodyPartName);
    }
  }
  else
  {
    mc_rtc::log::warning("[HumanRetargetingController] RetargetTaskList configuration is missing.");
  }

  // Setup managers
  if(config().has("RetargetManager"))
  {
    retargetManager_ = std::make_shared<RetargetManager>(this, config()("RetargetManager"));
  }
  else
  {
    mc_rtc::log::warning("[HumanRetargetingController] RetargetManager configuration is missing.");
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
    retargetManager_->update();
    centroidalManager_->update();
  }

  return mc_control::fsm::Controller::run();
}

void HumanRetargetingController::stop()
{
  // Clean up tasks
  for (const auto & [bodyPart, retargetTask] : retargetTasks_)
  {
    solver().removeTask(retargetTask);
  }

  // Clean up managers
  retargetManager_->stop();

  BaselineWalkingController::stop();
}
