// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/shuffleboard/Shuffleboard.h>
#include "subsystems/IntakeArmSubsystem.h"
#include "Constants.h"

#ifndef TESTBOARD

IntakeArmSubsystem::IntakeArmSubsystem() 

: m_intakeArmSparkMax{IntakeConstants::kIntakeArmMotorID, IntakeConstants::kIntakeMotorType} {

   #ifdef BURNINTAKEARMSPARKMAX

  rev::spark::SparkMaxConfig intakeArmSparkMaxConfig{};

  intakeArmSparkMaxConfig
  .VoltageCompensation(RobotConstants::kVoltageCompensationValue)
  .SetIdleMode(IntakeConstants::kIntakeMotorIdleMode)
  .SmartCurrentLimit(IntakeConstants::kIntakeMotorCurrentLimit.value());

  intakeArmSparkMaxConfig.closedLoop
  .Pid(IntakeConstants::kIntakeArmP, IntakeConstants::kIntakeArmI, IntakeConstants::kIntakeArmD)
  .OutputRange(IntakeConstants::kMinOutput, IntakeConstants::kMaxOutput)
  .SetFeedbackSensor(rev::spark::FeedbackSensor::kAbsoluteEncoder);

  m_intakeArmSparkMax.Configure(intakeArmSparkMaxConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);

auto nt_inst = nt::NetworkTableInstance::GetDefault();
auto nt_table = nt_inst.GetTable("Intake");

  nte_armEncoder = nt_table->GetEntry("Arm/Encoder");
//
//nte_coralInIntake = nt_table->GetEntry("Intake/Fuel in Intake");

  std::cout << "Flash Burned on IntakeArmSubsystem\r\n";
  #else
  std::cout << "Flash was not burned on IntakeArmSubsystem\r\n";
  #endif
}


void IntakeArmSubsystem::Periodic() {
  nte_armEncoder.SetDouble(m_intakeArmAbsoluteEncoder.GetPosition());
}


void IntakeArmSubsystem::SetIntakeArmMotorPower(double power) {
  // Sets the motor's power (between -1.0 and 1.0). 
  m_intakeArmSparkMax.Set(power);    

}

double IntakeArmSubsystem::GetDirection() {
  return m_intakeArmSparkMax.Get();
}
#endif //testboard

double IntakeArmSubsystem::GetEncoderValue() {
  return m_intakeArmAbsoluteEncoder.GetPosition();
}

void IntakeArmSubsystem::SetArmPosition(double setAngle) {
  if(setAngle > IntakeConstants::kIntakeUpperLimit) {
    setAngle = IntakeConstants::kIntakeUpperLimit;
  }
  if(setAngle < IntakeConstants::kIntakeLowerLimit) {
    setAngle = IntakeConstants::kIntakeLowerLimit;
  }

  m_intakeArmPID.SetReference(setAngle, rev::spark::SparkLowLevel::ControlType::kPosition);
}