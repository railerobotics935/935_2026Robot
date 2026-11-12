
#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>

#include "subsystems/AgitatorSubsystem.h"

#ifndef DISABLEINTAKE

class RunAgitator
  : public frc2::CommandHelper<frc2::Command, RunAgitator> {
public:
  /**
   * Creates a new RunAgitator.
   *
   * @param Agitator The pointer to the stager subsystem
   */
  explicit RunAgitator(AgitatorSubsystem* agitator);

  void Initialize() override;
  void End(bool interrupted) override;
  
private:
  AgitatorSubsystem* m_agitator;
};

#endif //DISABLEINTAKE 