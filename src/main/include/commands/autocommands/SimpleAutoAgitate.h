
#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/AgitatorSubsystem.h"

class SimpleAutoAgitate
  : public frc2::CommandHelper<frc2::Command, SimpleAutoAgitate> {
public:
  /**
   * Creates a new SimpleAutoAgitate.
   *
   * @param agitator The pointer to the agitator subsystem
   */
  explicit SimpleAutoAgitate(AgitatorSubsystem* agitator);

  void Initialize() override;
  void End(bool interrupted) override;
  
private:
  AgitatorSubsystem* m_agitator;
};