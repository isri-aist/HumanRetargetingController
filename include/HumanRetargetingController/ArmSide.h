#pragma once

#include <set>

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
std::string to_string(const HRC::ArmSide & armSide);
} // namespace std
