
#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>

#include "subsystems/StagerSubsystem.h"

class StagerStop
  : public frc2::CommandHelper<frc2::Command, StagerStop> {
public:
  /**
   * Creates a new SimpleStagerIntake.
   *
   * @param Stager The pointer to the stager subsystem
   */
  explicit StagerStop(StagerSubsystem* stager);

  void Initialize() override;
  void End(bool interrupted) override;
  
private:
  StagerSubsystem* m_stager;
};