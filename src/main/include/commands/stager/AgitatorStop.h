
#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>

#include "subsystems/AgitatorSubsystem.h"

#ifndef DISABLEINTAKE

class AgitatorStop
  : public frc2::CommandHelper<frc2::Command, AgitatorStop> {
public:
  /**
   * Creates a new AgitatorStop.
   *
   * @param Agitator The pointer to the stager subsystem
   */
  explicit AgitatorStop(AgitatorSubsystem* agitator, frc::XboxController* controller);

  void Initialize() override;
  void End(bool interrupted) override;
  
private:
  AgitatorSubsystem* m_agitator;
  frc::XboxController* m_controller;
};

#endif //DISABLEINTAKE 