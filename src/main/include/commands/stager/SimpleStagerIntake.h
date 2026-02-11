
#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>

#include "subsystems/StagerSubsystem.h"

class SimpleStagerIntake
  : public frc2::CommandHelper<frc2::Command, SimpleStagerIntake> {
public:
  /**
   * Creates a new SimpleStagerIntake.
   *
   * @param Stager The pointer to the stager subsystem
   */
  explicit SimpleStagerIntake(StagerSubsystem* stager);

  void Initialize() override;
  void End(bool interrupted) override;
  
private:
  StagerSubsystem* m_stager;
};