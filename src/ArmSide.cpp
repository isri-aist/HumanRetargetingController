#include <mc_rtc/logging.h>

#include <HumanRetargetingController/ArmSide.h>

using namespace HRC;

ArmSide HRC::strToArmSide(const std::string & armSideStr)
{
  if(armSideStr == "Left")
  {
    return ArmSide::Left;
  }
  else if(armSideStr == "Right")
  {
    return ArmSide::Right;
  }
  else
  {
    mc_rtc::log::error_and_throw("[strToArmSide] Unsupported ArmSide name: {}", armSideStr);
  }
}

ArmSide HRC::opposite(const ArmSide & armSide)
{
  if(armSide == ArmSide::Left)
  {
    return ArmSide::Right;
  }
  else // if(armSideStr == "Right")
  {
    return ArmSide::Left;
  }
}

int HRC::sign(const ArmSide & armSide)
{
  if(armSide == ArmSide::Left)
  {
    return 1;
  }
  else // if(armSideStr == "Right")
  {
    return -1;
  }
}

std::string std::to_string(const ArmSide & armSide)
{
  if(armSide == ArmSide::Left)
  {
    return std::string("Left");
  }
  else if(armSide == ArmSide::Right)
  {
    return std::string("Right");
  }
  else
  {
    mc_rtc::log::error_and_throw("[to_string] Unsupported armSide: {}", static_cast<int>(armSide));
  }
}
