
#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>

#include "subsystems/TurretYawSubsystem.h"

class SimpleRotateTurretYaw
  : public frc2::CommandHelper<frc2::Command, SimpleRotateTurretYaw> {
public:
  /**
   * Creates a new SimpleStagerIntake.
   *
   * @param Stager The pointer to the stager subsystem
   */
  explicit SimpleRotateTurretYaw(TurretYawSubsystem* intake, frc::XboxController *operatorController);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  
private:
  TurretYawSubsystem* m_turretYaw;
  frc::XboxController* m_operatorController;
};