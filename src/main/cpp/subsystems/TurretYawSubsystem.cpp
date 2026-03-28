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

//  turretYawSparkMaxConfig.closedLoop
//    .Pid(TurretYawConstants::kTurretYawP, TurretYawConstants::kTurretYawI, TurretYawConstants::kTurretYawD)
//    .OutputRange(TurretYawConstants::kTurretYawMinOutput, TurretYawConstants::kTurretYawMaxOutput)
//    .SetFeedbackSensor(rev::spark::FeedbackSensor::kDetachedRelativeEncoder);

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

  // TODO: check whether this is too early, not connected to driver station or field yet
  // may need to be moved to Auto init or TeleopInit methods
  auto alliance = frc::DriverStation::GetAlliance(); 
  if (alliance) {
    m_targetPose = FieldConstants::kRedHubCenter;
  }
  else {
    m_targetPose = FieldConstants::kBlueHubCenter;
  }

  auto nt_inst = nt::NetworkTableInstance::GetDefault();
  auto nt_table = nt_inst.GetTable("Turret");
  nte_turretEncoderPos = nt_table->GetEntry("Turret/EncoderPos");
  nte_turretYaw = nt_table->GetEntry("Turret/Yaw");
}

void TurretYawSubsystem::UpdateYawAiming(frc::Pose2d currentPose) {
  auto robotFieldRelYaw = currentPose.Rotation().Radians();

  auto deltaX = m_targetPose.X() - currentPose.X();
  auto deltaY = m_targetPose.Y() - currentPose.Y();
  double targetFieldRelYaw = atan(static_cast<double>(deltaY / deltaX));

  double setpTurretRobotRelYaw = targetFieldRelYaw - static_cast<double>(robotFieldRelYaw);
  double setpTurretRobotRelEncoderPos = -((setpTurretRobotRelYaw + (0.5 * std::numbers::pi)) * (22.5 / (2 * std::numbers::pi)));

  // Apply limits, when too close to an endpoint, update the setpoint to the opposite end of the rotation range
  if (setpTurretRobotRelEncoderPos >= m_turretMaxRotation) {
    setpTurretRobotRelEncoderPos = m_turretMinRotation + TurretYawConstants::kTurretYawDeadband;
  } else if (setpTurretRobotRelEncoderPos <= m_turretMinRotation) {
    setpTurretRobotRelEncoderPos = m_turretMaxRotation - TurretYawConstants::kTurretYawDeadband;
  }

  // Calculate the amount of power to apply to the turret motor
  double turretEncoderCurrentPos = m_turretYawEncoder.GetPosition();
  double turretMotorPwr = TurretYawConstants::kTurretGain * (setpTurretRobotRelEncoderPos - turretEncoderCurrentPos);

  // First limit power to an overall maximum
  if (turretMotorPwr > TurretYawConstants::kTurretYawMaxOutput)
    turretMotorPwr = TurretYawConstants::kTurretYawMaxOutput;
  else if (turretMotorPwr < -TurretYawConstants::kTurretYawMaxOutput)
    turretMotorPwr = -TurretYawConstants::kTurretYawMaxOutput;

  // And then slow down when reaching the physical endstop
  if ((m_turretMaxRotation - turretEncoderCurrentPos) < TurretYawConstants::kTurretSlowdownBand)
  {
    // Clockwise motion range
    turretMotorPwr = turretMotorPwr * ((m_turretMaxRotation - turretEncoderCurrentPos) / TurretYawConstants::kTurretSlowdownBand);
  }
  else if ((m_turretMinRotation - turretEncoderCurrentPos) > -TurretYawConstants::kTurretSlowdownBand)
  {
    // Counter Clockwise motion range
    turretMotorPwr = turretMotorPwr * ((m_turretMinRotation - turretEncoderCurrentPos) / -TurretYawConstants::kTurretSlowdownBand);
  }

  // And finally catch any end limit overrun attempts
  if ((turretMotorPwr > 0) && (turretEncoderCurrentPos >= m_turretMaxRotation))
  {
    // Clockwise motion range beyond endstop attempted
    turretMotorPwr = 0;
  }
  else if ((turretMotorPwr < 0) && (turretEncoderCurrentPos <= m_turretMinRotation))
  {
    // Counter Clockwise motion beyond endstop attempted
    turretMotorPwr = 0;
  }

  if (turretMotorPwr > TurretYawConstants::kTurretYawMaxOutput)
    turretMotorPwr = TurretYawConstants::kTurretYawMaxOutput;
  else if (turretMotorPwr < -TurretYawConstants::kTurretYawMaxOutput)
    turretMotorPwr = -TurretYawConstants::kTurretYawMaxOutput;

  m_turretYawSparkMax.Set(turretMotorPwr);

  if (coutCounter > 50)
  {
//    std::cout << "targetFieldRelYaw: " << targetFieldRelYaw << " setpTurretRobotRelYaw: " << setpTurretRobotRelYaw << " setpTurretRobotRelEncoderPos: " << setpTurretRobotRelEncoderPos << std::endl;
    std::cout << "targetFieldRelYaw: " << targetFieldRelYaw << " setpTurretRobotRelYaw: " << setpTurretRobotRelYaw << " setpTurretPwr: " << turretMotorPwr << std::endl;
    coutCounter = 0;
  }
  else
  {
    coutCounter++;
  }

//  if ((m_turretYawEncoder.GetPosition() < m_turretMaxRotation) && (m_turretYawEncoder.GetPosition() > m_turretMinRotation))
//  {
//    m_TurretYawPID.SetReference(setpTurretRobotRelEncoderPos, rev::spark::SparkLowLevel::ControlType::kPosition);
//  }
//  else
//  {
//    m_turretYawSparkMax.Set(0.0);
//  }
}

void TurretYawSubsystem::Periodic() {
  double turretEncoderPos = m_turretYawEncoder.GetPosition();

  // Counterclockwise is NEGATIVE
  if (!m_YawLimitSwitch.Get())
  {
    if (GetDirection() > 0.02)
    {
      // turret rotates clockwise
      // only update the maxRotation when in the positive range of the encoder
      if (turretEncoderPos > 0.0)
      {
        m_turretMaxRotation = turretEncoderPos - 2.0;
        std::cout << "Turret MAX Rotation set to: " << m_turretMaxRotation << std::endl;
      }
    }
    else if (GetDirection() < -0.02)
    {
      // turret rotates counterclockwise
      // only update the minRotation when in the negative range of the encoder
      if (turretEncoderPos < 0.0)
      {
        m_turretMinRotation = turretEncoderPos + 2.0;
        std::cout << "Turret MIN Rotation set to: " << m_turretMinRotation << std::endl;
      }
    }
  }

  nte_turretEncoderPos.SetDouble(turretEncoderPos);
  double turretRobotRelYaw = (-turretEncoderPos / 22.5 * (2 * std::numbers::pi)) - (0.5 * std::numbers::pi);
  nte_turretYaw.SetDouble(turretRobotRelYaw);
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
  return (-m_turretYawEncoder.GetPosition() / 22.5 * (2 * std::numbers::pi)) - (0.5 * std::numbers::pi);
}

//#endif //testboard