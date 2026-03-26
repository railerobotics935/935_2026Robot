
#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/ShooterSubsystem.h"

class SimpleAutoShoot
  : public frc2::CommandHelper<frc2::Command, SimpleAutoShoot> {
public:
  /**
   * Creates a new SimpleAutoShoot.
   *
   * @param shooter The pointer to the shooter subsystem
   */
  explicit SimpleAutoShoot(ShooterSubsystem* shooter);

  void Initialize() override;
  void End(bool interrupted) override;
  
private:
  ShooterSubsystem* m_shooter;
};