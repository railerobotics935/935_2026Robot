
#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>

#include "subsystems/TurretYawSubsystem.h"
#include "subsystems/DriveSubsystem.h"

//#ifndef TESTBOARD

class SimpleTrackingRotate
  : public frc2::CommandHelper<frc2::Command, SimpleTrackingRotate> {
public:
  /**
   * Creates a new SimpleStagerIntake.
   *
   * @param Stager The pointer to the stager subsystem
   */
  explicit SimpleTrackingRotate(TurretYawSubsystem* intake, DriveSubsystem *drive);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  
private:
  TurretYawSubsystem* m_turretYaw;
  DriveSubsystem* m_drive;
  frc::XboxController* m_operatorController;
};

//#endif //TESTBOARD