#pragma once

#include <mc_rtc/logging.h>
#include <set>
#include <string>

namespace HRC
{
/** \brief Left/right side of the arm. */
enum class ArmSide
{
  //! Left arm
  Left = 0,

  //! Right arm
  Right
};

namespace ArmSides
{
//! Both arms
const std::set<ArmSide> Both = {ArmSide::Left, ArmSide::Right};
} // namespace ArmSides

/** \brief Convert string to arm side. */
ArmSide strToArmSide(const std::string & armSideStr);

/** \brief Get the opposite arm side. */
ArmSide opposite(const ArmSide & armSide);

/** \brief Get the sign of arm side.

    Positive for left arm side, negative for right arm side.
*/
int sign(const ArmSide & armSide);
} // namespace HRC

namespace std
{
/** \brief Convert armSide to string. */
inline string to_string(const HRC::ArmSide & armSide)
{
  if(armSide == HRC::ArmSide::Left)
  {
    return std::string("Left");
  }
  else if(armSide == HRC::ArmSide::Right)
  {
    return std::string("Right");
  }
  else
  {
    mc_rtc::log::error_and_throw("[to_string] Unsupported armSide: {}", static_cast<int>(armSide));
  }
}
} // namespace std
