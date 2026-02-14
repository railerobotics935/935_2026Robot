
#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>

#include "Constants.h"
#include "subsystems/TurretPitchSubsystem.h"

#ifndef TESTBOARD

class SimpleMoveTurretPitch
  : public frc2::CommandHelper<frc2::Command, SimpleMoveTurretPitch> {
public:
  /**
   * Creates a new SimpleMoveTurretPitch.
   *
   * @param Stager The pointer to the stager subsystem
   */
  explicit SimpleMoveTurretPitch(TurretPitchSubsystem* turretPitch, frc::XboxController* operatorController);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  
private:
  TurretPitchSubsystem* m_turretPitch;
  frc::XboxController* m_operatorController;
};
#endif //TESTBOARD