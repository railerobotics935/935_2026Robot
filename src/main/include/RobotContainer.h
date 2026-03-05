// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>

#include "Constants.h"
#include "subsystems/ExampleSubsystem.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/StagerSubsystem.h"
#include "subsystems/TurretYawSubsystem.h"
#include "subsystems/TurretPitchSubsystem.h"
#include "subsystems/IntakeSubsystem.h"

#include "commands/drive/DriveWithController.h"
#include "commands/shooter/SimpleShoot.h"
#include "commands/shooter/StopShooter.h"
#include "commands/stager/SimpleStagerIntake.h"
#include "commands/stager/StagerStop.h"
#include "commands/turretyaw/SimpleRotateTurretYaw.h"
#include "commands/turretpitch/SimpleMoveTurretPitch.h"
#include "commands/intake/SimpleIntake.h"
#include "commands/intake/SimpleOuttake.h"
#include "commands/intake/StopIntake.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();

 private:
  // Replace with CommandPS4Controller or CommandJoystick if needed
  //frc2::CommandXboxController m_driverController{
  //    OperatorConstants::kDriverControllerPort};

  frc::XboxController m_driveController{OperatorConstants::kDriverControllerPort};
  frc::XboxController m_operatorController{OperatorConstants::kOperatorControllerPort};

  // The robot's subsystems are defined here
  ExampleSubsystem m_subsystem;
  DriveSubsystem m_driveSubsystem;
  #ifndef TESTBOARD
  ShooterSubsystem m_shooterSubsystem;
  StagerSubsystem m_stagerSubsystem;
  TurretYawSubsystem m_turretYawSubsystem;
  TurretPitchSubsystem m_turretPitchSubsystem;
  IntakeSubsystem m_intakeSubsystem;
  #endif //testboard

  void ConfigureBindings();

  // Commands
  DriveWithController m_driveWithController{&m_driveSubsystem, &m_driveController};
  #ifndef TESTBOARD
  SimpleRotateTurretYaw m_simpleRotateTurretYaw{&m_turretYawSubsystem, &m_operatorController};
  SimpleMoveTurretPitch m_simpleMoveTurretPitch{&m_turretPitchSubsystem, &m_operatorController};
  StagerStop m_stagerStop{&m_stagerSubsystem, &m_operatorController};
  StopIntake m_stopIntake{&m_intakeSubsystem, &m_operatorController};
  StopShooter m_stopShooter{&m_shooterSubsystem, &m_operatorController};
  #endif //testboard

};
