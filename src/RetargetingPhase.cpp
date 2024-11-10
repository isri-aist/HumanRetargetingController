#include <mc_rtc/logging.h>

#include <HumanRetargetingController/RetargetingPhase.h>

using namespace HRC;

std::string std::to_string(const RetargetingPhase & phase)
{
  if(phase == RetargetingPhase::Disabled)
  {
    return std::string("Disabled");
  }
  else if(phase == RetargetingPhase::Enabled)
  {
    return std::string("Enabled");
  }
  else if(phase == RetargetingPhase::Frozen)
  {
    return std::string("Frozen");
  }
  else
  {
    mc_rtc::log::error_and_throw("[to_string] Unsupported retargeting phase: {}",
                                 std::to_string(static_cast<int>(phase)));
  }
}
