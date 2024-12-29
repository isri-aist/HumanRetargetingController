#include <HumanRetargetingController/HumanRetargetingController.h>
#include <HumanRetargetingController/State.h>

using namespace HRC;

void State::start(mc_control::fsm::Controller & _ctl)
{
  ctlPtr_ = &static_cast<HumanRetargetingController &>(_ctl);
}
