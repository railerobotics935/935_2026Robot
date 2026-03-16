
#pragma once

#ifndef TESTBOARD

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>

#include "subsystems/IntakeArmSubsystem.h"

class SimpleStopArm
  : public frc2::CommandHelper<frc2::Command, SimpleStopArm> {
public:
  /**
   * Creates a new SimpleLowerArm.
   *
   * @param intakeArm The pointer to the intakeArm subsystem
   */
  explicit SimpleStopArm(IntakeArmSubsystem* intakeArm);

  void Initialize() override;
  void End(bool interrupted) override;
  
private:
  IntakeArmSubsystem* m_intakeArm;
};
#endif //TESTBOARD