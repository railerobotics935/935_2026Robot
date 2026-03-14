// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <rev/SparkMax.h>
#include <frc/DigitalInput.h>
#include <iostream>
#include <Constants.h>



class TurretPitchSubsystem : public frc2::SubsystemBase {
 public:
  /**
   * Picks up game pieces
  */
  TurretPitchSubsystem();

  // Sets the motor's power (between -1.0 and 1.0).
  
  void Periodic() override;

  
  void SetTurretPitchMotorPower(double power);

  /**
   * @return Direction intake motor is moving
   */
  double GetDirection();

  /**
   * @return If light sensor has detected a coral
   */
  //bool CoralInShooter();

  bool TurretPitchAtZero();

 private:

  void ConfigureSparkMax();

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  // Motor Controllers
  rev::spark::SparkMax m_turretPitchSparkMax;

  // Encoders
  rev::spark::SparkRelativeEncoder m_turretPitchEncoder = m_turretPitchSparkMax.GetEncoder();

  // Limit Switch 
  frc::DigitalInput m_PitchLimitSwitch{TurretPitchConstants::kPitchLimitSwitchPort};

  // PID
  rev::spark::SparkClosedLoopController m_TurretPitchPID = m_turretPitchSparkMax.GetClosedLoopController();

  // Light Sensor is a digital input in the DIO port (digital input output)
  //frc::DigitalInput m_lightSensor{ShooterConstants::kLightSensorID};


  //Network Table Entry
  //nt::NetworkTableEntry nte_coralInShooter;
  nt::NetworkTableEntry nte_turretPitchAngle;
};
