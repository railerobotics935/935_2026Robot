
#pragma once

#ifndef TESTBOARD

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>

#include "subsystems/IntakeArmSubsystem.h"

class SimpleLowerArm
  : public frc2::CommandHelper<frc2::Command, SimpleLowerArm> {
public:
  /**
   * Creates a new SimpleLowerArm.
   *
   * @param intakeArm The pointer to the intakeArm subsystem
   */
  explicit SimpleLowerArm(IntakeArmSubsystem* intakeArm);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  
private:
  IntakeArmSubsystem* m_intakeArm;
  double m_currentArmPosition; 
};
#endif //TESTBOARD