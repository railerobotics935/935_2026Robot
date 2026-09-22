
#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>

#include "subsystems/StagerSubsystem.h"
#include "Constants.h"

#ifndef DISABLEINTAKE

class StagerOuttake
  : public frc2::CommandHelper<frc2::Command, StagerOuttake> {
public:
  /**
   * Creates a new StagerOuttake.
   *
   * @param Stager The pointer to the stager subsystem
   */
  explicit StagerOuttake(StagerSubsystem* stager);

  void Initialize() override;
  void End(bool interrupted) override;
  
private:
  StagerSubsystem* m_stager;
};

#endif //DISABLEINTAKE