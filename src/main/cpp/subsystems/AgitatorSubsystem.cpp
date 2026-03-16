// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/shuffleboard/Shuffleboard.h>
#include "subsystems/AgitatorSubsystem.h"
#include "Constants.h"

#ifndef TESTBOARD

AgitatorSubsystem::AgitatorSubsystem() 

: m_agitatorSparkMax{AgitatorConstants::kAgitatorMotorID, AgitatorConstants::kAgitatorMotorType} {

   #ifdef BURNAGITATORSPARKMAX

  rev::spark::SparkMaxConfig agitatorSparkMaxConfig{};

  agitatorSparkMaxConfig
  .VoltageCompensation(RobotConstants::kVoltageCompensationValue)
  .SetIdleMode(AgitatorConstants::kAgitatorMotorIdleMode)
  .SmartCurrentLimit(AgitatorConstants::kAgitatorMotorCurrentLimit.value());

  m_agitatorSparkMax.Configure(agitatorSparkMaxConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);

//auto nt_inst = nt::NetworkTableInstance::GetDefault();
//auto nt_table = nt_inst.GetTable("Agitator");
//
//nte_coralInAgitator = nt_table->GetEntry("Agitator/Fuel in Agitator");

  std::cout << "Flash Burned on Agitator subsystem\r\n";
  #else
  std::cout << "Flash was not burned on Agitator subsystem\r\n";
  #endif
}


void AgitatorSubsystem::Periodic() {
  
}


void AgitatorSubsystem::SetAgitatorMotorPower(double power) {
  // Sets the motor's power (between -1.0 and 1.0). 
  m_agitatorSparkMax.Set(power);    

}

double AgitatorSubsystem::GetDirection() {
  return m_agitatorSparkMax.Get();
}
#endif //testboard