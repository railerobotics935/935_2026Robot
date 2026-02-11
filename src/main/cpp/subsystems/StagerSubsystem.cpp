// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/shuffleboard/Shuffleboard.h>
#include "subsystems/StagerSubsystem.h"
#include "Constants.h"

StagerSubsystem::StagerSubsystem() 

: m_stagerSparkMax{StagerConstants::kStagerShooterMotorID, StagerConstants::kStagerMotorType}
 {


   #ifdef BURNSHOOTERSPARKMAX

  rev::spark::SparkMaxConfig stagerSparkMaxConfig{};

  stagerSparkMaxConfig
  .VoltageCompensation(RobotConstants::kVoltageCompensationValue)
  .SetIdleMode(StagerConstants::kStagerMotorIdleMode)
  .SmartCurrentLimit(StagerConstants::kStagerMotorCurrentLimit.value());

  m_stagerSparkMax.Configure(stagerSparkMaxConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);

  
//auto nt_inst = nt::NetworkTableInstance::GetDefault();
//auto nt_table = nt_inst.GetTable("Shooter");
//
//nte_coralInShooter = nt_table->GetEntry("Shooter/Fuel in Shooter");

  std::cout << "Flash Burned on stager subsystem\r\n";
  #else
  std::cout << "Flash was not burned on stager subsystem\r\n";
  #endif
}

/*
void ShooterSubsystem::Periodic() {
  nte_coralInShooter.SetBoolean(CoralInShooter());
}
*/

void StagerSubsystem::SetStagerMotorPower(double power) {
  // Sets the motor's power (between -1.0 and 1.0). 
  m_stagerSparkMax.Set(power);    

}

double StagerSubsystem::GetDirection() {
  return m_stagerSparkMax.Get();
}
