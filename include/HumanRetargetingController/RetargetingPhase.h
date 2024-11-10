#pragma once

#include <string>

namespace HRC
{
/** \brief Retargeting phase. */
enum class RetargetingPhase
{
  //! Retargeting task disabled
  Disabled = 0,

  //! Retargeting task enabled
  Enabled,

  //! Retargeting task enabled but with fixed target
  Frozen,
};
} // namespace HRC

namespace std
{
/** \brief Convert retargeting phase to string. */
std::string to_string(const HRC::RetargetingPhase & phase);
} // namespace std
