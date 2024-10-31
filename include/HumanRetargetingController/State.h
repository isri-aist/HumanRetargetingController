#pragma once

#include <mc_control/fsm/State.h>

namespace HRC
{
class HumanRetargetingController;

/** \brief FSM State with utility functions. */
struct State : mc_control::fsm::State
{
public:
  /** \brief Start. */
  void start(mc_control::fsm::Controller & _ctl) override;

protected:
  /** \brief Const accessor to the controller. */
  inline const HumanRetargetingController & ctl() const
  {
    return *ctlPtr_;
  }

  /** \brief Accessor to the controller. */
  inline HumanRetargetingController & ctl()
  {
    return *ctlPtr_;
  }

protected:
  //! Pointer to controller
  HumanRetargetingController * ctlPtr_ = nullptr;
};
} // namespace HRC
