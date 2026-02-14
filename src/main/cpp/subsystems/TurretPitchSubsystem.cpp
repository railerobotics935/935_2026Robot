// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/shuffleboard/Shuffleboard.h>
#include "subsystems/TurretPitchSubsystem.h"
#include "Constants.h"

#ifndef TESTBOARD

TurretPitchSubsystem::TurretPitchSubsystem() 

: m_turretPitchSparkMax{TurretPitchConstants::kTurretPitchMotorID, TurretPitchConstants::kTurretPitchMotorType}
 {


   #ifdef BURNSHOOTERSPARKMAX

  rev::spark::SparkMaxConfig turretPitchSparkMaxConfig{};

  turretPitchSparkMaxConfig
  .VoltageCompensation(RobotConstants::kVoltageCompensationValue)
  .SetIdleMode(TurretPitchConstants::kTurretPitchMotorIdleMode)
  .SmartCurrentLimit(TurretPitchConstants::kTurretPitchMotorCurrentLimit.value());

  m_turretPitchSparkMax.Configure(turretPitchSparkMaxConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);

  
auto nt_inst = nt::NetworkTableInstance::GetDefault();
auto nt_table = nt_inst.GetTable("Turret");

nte_turretPitchAngle = nt_table->GetEntry("Turret/Turret Pitch");
//auto nt_table = nt_inst.GetTable("Shooter");
//
//nte_coralInShooter = nt_table->GetEntry("Shooter/Fuel in Shooter");

  std::cout << "Flash Burned on TurretPitch subsystem\r\n";
  #else
  std::cout << "Flash was not burned on TurretPitch subsystem\r\n";
  #endif
}


void TurretPitchSubsystem::Periodic() {
  nte_turretPitchAngle.SetDouble(m_pitchEncoder.GetPosition()); // Up is NEGATIVE
}


void TurretPitchSubsystem::SetTurretPitchMotorPower(double power) {
  // Sets the motor's power (between -1.0 and 1.0). 
  m_turretPitchSparkMax.Set(power);    

}

double TurretPitchSubsystem::GetDirection() {
  return m_turretPitchSparkMax.Get();
}
#endif //Testboard