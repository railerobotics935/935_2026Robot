// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/shuffleboard/Shuffleboard.h>
#include "subsystems/TurretYawSubsystem.h"
#include "Constants.h"

//#ifndef TESTBOARD

TurretYawSubsystem::TurretYawSubsystem() 
  : m_turretYawSparkMax{TurretYawConstants::kTurretYawMotorID, TurretYawConstants::kTurretYawMotorType}
{
#ifdef BURNSHOOTERSPARKMAX
  rev::spark::SparkMaxConfig turretYawSparkMaxConfig{};
  turretYawSparkMaxConfig
    .VoltageCompensation(RobotConstants::kVoltageCompensationValue)
    .SetIdleMode(TurretYawConstants::kTurretYawMotorIdleMode)
    .SmartCurrentLimit(TurretYawConstants::kTurretYawMotorCurrentLimit.value());
  m_turretYawSparkMax.Configure(turretYawSparkMaxConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);

//  auto nt_inst = nt::NetworkTableInstance::GetDefault();
//  auto nt_table = nt_inst.GetTable("Turret");
//  nte_turretYawAngle = nt_table->GetEntry("Turret/Turret Yaw");
  //auto nt_table = nt_inst.GetTable("Shooter");
  //
  //nte_coralInShooter = nt_table->GetEntry("Shooter/Fuel in Shooter");
  std::cout << "Flash Burned on TurretYaw subsystem\r\n";
#else
  std::cout << "Flash was not burned on TurretYaw subsystem\r\n";
#endif
  m_turretYawEncoder.SetPosition(0.0);
}

bool TurretYawSubsystem::TurretYawAtZero() {
  return !m_YawLimitSwitch.Get();
}

void TurretYawSubsystem::Periodic() {
//  nte_turretYawAngle.SetDouble(m_turretYawEncoder.GetPosition());
  // Counterclockwise is NEGATIVE

  if (TurretYawAtZero())
  {
    if (GetDirection() > 0.02)
    {
      // turret rotates clockwise

      // only update the maxRotation when in the positive range of the encoder
      if (m_turretYawEncoder.GetPosition() > 0.0)
      {
        m_turretMaxRotation = m_turretYawEncoder.GetPosition() - 2.0;
        std::cout << "Turret MAX Rotation set to: " << m_turretMaxRotation << std::endl;
      }
    }
    else if (GetDirection() < -0.02)
    {
      // turret rotates counterclockwise

      // only update the minRotation when in the negative range of the encoder
      if (m_turretYawEncoder.GetPosition() < 0.0)
      {
        m_turretMinRotation = m_turretYawEncoder.GetPosition() + 2.0;
        std::cout << "Turret MIN Rotation set to: " << m_turretMinRotation << std::endl;
      }
    }
  }
}

void TurretYawSubsystem::SetTurretYawMotorPower(double power) {
  // Sets the motor's power (between -1.0 and 1.0)
  if (power > 0.0)
  {
    // turret is requested to go clockwise
    if (m_turretYawEncoder.GetPosition() >= m_turretMaxRotation)
      power = 0.0;
  }
  else
  {
    // turret is requested to go counterclockwise
    if (m_turretYawEncoder.GetPosition() <= m_turretMinRotation)
      power = 0.0;
  }
  m_turretYawSparkMax.Set(power);    
}

double TurretYawSubsystem::GetDirection() {
  return m_turretYawSparkMax.Get();
}

double TurretYawSubsystem::GetCurrentYaw() {
  return m_turretYawEncoder.GetPosition() - m_turretMaxRotation;
}
//#endif //testboard