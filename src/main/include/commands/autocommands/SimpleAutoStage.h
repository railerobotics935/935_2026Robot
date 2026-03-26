
#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/StagerSubsystem.h"

class SimpleAutoStage
  : public frc2::CommandHelper<frc2::Command, SimpleAutoStage> {
public:
  /**
   * Creates a new SimpleAutoStage.
   *
   * @param stager The pointer to the stager subsystem
   */
  explicit SimpleAutoStage(StagerSubsystem* stager);

  void Initialize() override;
  void End(bool interrupted) override;
  
private:
  StagerSubsystem* m_stager;
};