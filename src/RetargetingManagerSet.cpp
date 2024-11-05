#include <mc_rtc/gui/ArrayInput.h>
#include <mc_rtc/gui/Label.h>

#include <HumanRetargetingController/RetargetingManagerSet.h>
#include <HumanRetargetingController/HumanRetargetingController.h>

using namespace HRC;

void RetargetingManagerSet::Configuration::load(const mc_rtc::Configuration & mcRtcConfig)
{
  mcRtcConfig("name", name);
}

RetargetingManagerSet::RetargetingManagerSet(HumanRetargetingController * ctlPtr, const mc_rtc::Configuration & mcRtcConfig)
: ctlPtr_(ctlPtr)
{
  config_.load(mcRtcConfig);

  for(const mc_rtc::Configuration & retargetingManagerConfig : mcRtcConfig("RetargetingManagerList", mc_rtc::Configuration()))
  {
    this->emplace(retargetingManagerConfig("bodyPart"), std::make_shared<RetargetingManager>(ctlPtr, retargetingManagerConfig));
  }
}

void RetargetingManagerSet::reset()
{
  for(const auto & limbManagerKV : *this)
  {
    limbManagerKV.second->reset();
  }
}

void RetargetingManagerSet::update()
{
  for(const auto & limbManagerKV : *this)
  {
    limbManagerKV.second->update();
  }

  if(!isTaskEnabled_)
  {
    bool isFirstMsgObtained = true;
    for(const auto & limbManagerKV : *this)
    {
      if(isFirstMsgObtained && !limbManagerKV.second->isFirstMsgObtained())
      {
        isFirstMsgObtained = false;
      }
    }

    if(isFirstMsgObtained)
    {
      isTaskEnabled_ = true;
      mc_rtc::log::success("[RetargetingManagerSet] Enable retargeting tasks.");

      for(const auto & limbManagerKV : *this)
      {
        limbManagerKV.second->enableTask();
      }
    }
  }
}

void RetargetingManagerSet::stop()
{
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

  // TODO
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
