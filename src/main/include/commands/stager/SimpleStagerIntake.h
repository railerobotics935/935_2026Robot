
#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>

#include "subsystems/StagerSubsystem.h"
#include "subsystems/AgitatorSubsystem.h"
#include "Constants.h"

#ifndef TESTBOARD

class SimpleStagerIntake
  : public frc2::CommandHelper<frc2::Command, SimpleStagerIntake> {
public:
  /**
   * Creates a new SimpleStagerIntake.
   *
   * @param Stager The pointer to the stager subsystem
   * @param Agitator The pointer to the agitator subsystem
   * @param operatorController The pointer to the operator controller
   */
  explicit SimpleStagerIntake(StagerSubsystem* stager, AgitatorSubsystem* agitator, frc::XboxController* operatorController);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  
private:
  StagerSubsystem* m_stager;
  AgitatorSubsystem* m_agitator;
  frc::XboxController* m_operatorController;
};

#endif //Tesboard