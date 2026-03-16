// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/shuffleboard/Shuffleboard.h>
#include "subsystems/IntakeSubsystem.h"
#include "Constants.h"

#ifndef TESTBOARD

IntakeSubsystem::IntakeSubsystem() 

: m_intakeFrontSparkMax{IntakeConstants::kIntakeFrontMotorID, IntakeConstants::kIntakeMotorType}, m_intakeRearSparkMax{IntakeConstants::kIntakeRearMotorID, IntakeConstants::kIntakeMotorType} {

   #ifdef BURNINTAKESPARKMAX

  rev::spark::SparkMaxConfig intakeFrontSparkMaxConfig{};

  intakeFrontSparkMaxConfig
  .VoltageCompensation(RobotConstants::kVoltageCompensationValue)
  .SetIdleMode(IntakeConstants::kIntakeMotorIdleMode)
  .SmartCurrentLimit(IntakeConstants::kIntakeMotorCurrentLimit.value());

  m_intakeFrontSparkMax.Configure(intakeFrontSparkMaxConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);

  rev::spark::SparkMaxConfig intakeRearSparkMaxConfig{};

  intakeRearSparkMaxConfig
  .VoltageCompensation(RobotConstants::kVoltageCompensationValue)
  .SetIdleMode(IntakeConstants::kIntakeMotorIdleMode)
  .SmartCurrentLimit(IntakeConstants::kIntakeMotorCurrentLimit.value())
  .Follow(IntakeConstants::kIntakeFrontMotorID, false);

  m_intakeRearSparkMax.Configure(intakeRearSparkMaxConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);

//auto nt_inst = nt::NetworkTableInstance::GetDefault();
//auto nt_table = nt_inst.GetTable("Intake");
//
//nte_coralInIntake = nt_table->GetEntry("Intake/Fuel in Intake");

  std::cout << "Flash Burned on Intake subsystem\r\n";
  #else
  std::cout << "Flash was not burned on Intake subsystem\r\n";
  #endif
}


void IntakeSubsystem::Periodic() {
  
}


void IntakeSubsystem::SetIntakeMotorPower(double power) {
  // Sets the motor's power (between -1.0 and 1.0). 
  m_intakeFrontSparkMax.Set(power);    

}

double IntakeSubsystem::GetDirection() {
  return m_intakeFrontSparkMax.Get();
}
#endif //testboard