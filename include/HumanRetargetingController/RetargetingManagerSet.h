#pragma once

#include <unordered_set>

#include <HumanRetargetingController/RetargetingManager.h>

namespace HRC
{
/** \brief Set of RetargetingManager. */
class RetargetingManagerSet : public std::unordered_map<std::string, std::shared_ptr<RetargetingManager>>
{
  /** \brief Configuration. */
  struct Configuration
  {
    //! Name
    std::string name = "RetargetingManagerSet";

    /** \brief Load mc_rtc configuration.
        \param mcRtcConfig mc_rtc configuration
    */
    void load(const mc_rtc::Configuration & mcRtcConfig);
  };

public:
  /** \brief Constructor.
      \param ctlPtr pointer to controller
      \param mcRtcConfig mc_rtc configuration
  */
  RetargetingManagerSet(HumanRetargetingController * ctlPtr, const mc_rtc::Configuration & mcRtcConfig = {});

  /** \brief Reset.

      This method should be called once when controller is reset.
  */
  void reset();

  /** \brief Update.

      This method should be called once every control cycle.
  */
  void update();

  /** \brief Stop.

      This method should be called once when stopping the controller.
  */
  void stop();

  /** \brief Const accessor to the configuration. */
  inline const Configuration & config() const noexcept
  {
    return config_;
  }

  /** \brief Add entries to the GUI. */
  void addToGUI(mc_rtc::gui::StateBuilder & gui);

  /** \brief Remove entries from the GUI. */
  void removeFromGUI(mc_rtc::gui::StateBuilder & gui);

  /** \brief Add entries to the logger. */
  void addToLogger(mc_rtc::Logger & logger);

  /** \brief Remove entries from the logger. */
  void removeFromLogger(mc_rtc::Logger & logger);

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
  //! Configuration
  Configuration config_;

  //! Pointer to controller
  HumanRetargetingController * ctlPtr_ = nullptr;

  //! Whether the retargeting task is enabled
  bool isTaskEnabled_ = false;
};
} // namespace HRC