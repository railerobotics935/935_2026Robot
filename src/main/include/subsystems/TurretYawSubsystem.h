// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/geometry/Pose2d.h>
#include <frc/DriverStation.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <rev/SparkMax.h>
#include <frc/DigitalInput.h>
#include <iostream>
#include <Constants.h>
#include <math.h>

//#ifndef TESTBOARD

class TurretYawSubsystem : public frc2::SubsystemBase {
 public:
  TurretYawSubsystem();

  void Periodic() override;
  
  // Sets the motor's power (between -1.0 and 1.0).
  void SetTurretYawMotorPower(double power);

  /**
   * @return Direction turret yaw motor is moving
   */
  double GetDirection();

  /**
   * @return Robot relative turret yaw
   */
  double GetCurrentYaw();

  void UpdateYawAiming(frc::Pose2d currentPose);

 private:

  void ConfigureSparkMax();

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  // Motor Controllers
  rev::spark::SparkMax m_turretYawSparkMax;

  // Encoders
  rev::spark::SparkRelativeEncoder m_turretYawEncoder = m_turretYawSparkMax.GetEncoder();

  //PID Controller
//  rev::spark::SparkClosedLoopController m_TurretYawPID = m_turretYawSparkMax.GetClosedLoopController();

  // Light Sensor is a digital input in the DIO port (digital input output)
  //frc::DigitalInput m_lightSensor{ShooterConstants::kLightSensorID};

  //Network Table Entry
  nt::NetworkTableEntry nte_turretEncoderPos;
  nt::NetworkTableEntry nte_turretYaw;

  // Limit switch is a digital input in the DIO port (digital input output)
  frc::DigitalInput m_YawLimitSwitch{TurretYawConstants::kYawLimitSwitchPort};

  // defaults based on turret placed at zero point when powered off
  double m_turretMaxRotation = 11.0;
  double m_turretMinRotation = -11.86;

  frc::Pose2d m_targetPose;

  uint8_t coutCounter = 0;
};

//#endif //Tesboard