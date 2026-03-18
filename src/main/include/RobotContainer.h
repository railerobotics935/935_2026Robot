// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc/smartdashboard/SendableChooser.h>

#include "Constants.h"
#include "subsystems/ExampleSubsystem.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/StagerSubsystem.h"
#include "subsystems/TurretYawSubsystem.h"
#include "subsystems/TurretPitchSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/IntakeArmSubsystem.h"
#include "subsystems/AgitatorSubsystem.h"

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
#include "commands/intakearm/SimpleLowerArm.h"
#include "commands/intakearm/SimpleRaiseArm.h"
#include "commands/intakearm/SimpleStopArm.h"
#include "commands/stager/AgitatorStop.h"

#include <frc2/command/Commands.h>

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
  TurretPitchSubsystem m_turretPitchSubsystem;
  #ifndef TESTBOARD
  ShooterSubsystem m_shooterSubsystem;
  StagerSubsystem m_stagerSubsystem;
  IntakeSubsystem m_intakeSubsystem;
  #endif //testboard
  TurretYawSubsystem m_turretYawSubsystem;
  IntakeArmSubsystem m_intakeArmSubsystem;
  AgitatorSubsystem m_agitatorSubsystem;
  


  void ConfigureBindings();

  frc::SendableChooser<std::string> m_autoChooser;

  // Commands
  DriveWithController m_driveWithController{&m_driveSubsystem, &m_driveController};
  SimpleRotateTurretYaw m_simpleRotateTurretYaw{&m_turretYawSubsystem, &m_operatorController};
  SimpleMoveTurretPitch m_simpleMoveTurretPitch{&m_turretPitchSubsystem, &m_operatorController};
  #ifndef TESTBOARD
  StagerStop m_stagerStop{&m_stagerSubsystem, &m_operatorController};
  StopIntake m_stopIntake{&m_intakeSubsystem, &m_operatorController};
  SimpleIntake m_simpleIntake{&m_intakeSubsystem};
  SimpleOuttake m_simpleOuttake{&m_intakeSubsystem};
  StopShooter m_stopShooter{&m_shooterSubsystem, &m_operatorController};
  SimpleShoot m_simpleShoot{&m_shooterSubsystem, &m_operatorController};
  #endif //testboard
  SimpleStopArm m_simpleStopArm{&m_intakeArmSubsystem};
  SimpleLowerArm m_simpleLowerArm{&m_intakeArmSubsystem};
  SimpleRaiseArm m_simpleRaiseArm{&m_intakeArmSubsystem};
  AgitatorStop m_agitatorStop{&m_agitatorSubsystem};
  SimpleStagerIntake m_simpleStage{&m_stagerSubsystem, &m_agitatorSubsystem, &m_operatorController};

};
