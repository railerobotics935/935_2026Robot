// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/JoystickButton.h>

#include <pathplanner/lib/auto/NamedCommands.h>
#include "pathplanner/lib/commands/PathPlannerAuto.h"
#include <frc/shuffleboard/Shuffleboard.h>

#include "commands/Autos.h"
#include "commands/ExampleCommand.h"

using namespace pathplanner;

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureBindings();

  // Set Default Commands for Subsystems
  m_driveSubsystem.SetDefaultCommand(std::move(m_driveWithController));
  #ifndef TESTBOARD
  m_turretPitchSubsystem.SetDefaultCommand(std::move(m_simpleMoveTurretPitch));
  m_stagerSubsystem.SetDefaultCommand(std::move(m_simpleStageIntake));
  m_shooterSubsystem.SetDefaultCommand(std::move(m_simpleShoot));
  m_intakeSubsystem.SetDefaultCommand(std::move(m_stopIntake)); 
  #endif //testboard
  m_turretYawSubsystem.SetDefaultCommand(std::move(m_simpleRotateTurretYaw));
  m_intakeArmSubsystem.SetDefaultCommand(std::move(m_simpleStopArm));

  // auto commands for pathplanner
  NamedCommands::registerCommand("Extend Intake", std::move(m_simpleLowerArm).ToPtr());
  NamedCommands::registerCommand("Intake Fuel", std::move(m_simpleIntake).ToPtr());
  NamedCommands::registerCommand("Stop Intake", std::move(m_stopIntake).ToPtr());
  NamedCommands::registerCommand("Outtake Fuel", std::move(m_simpleOuttake).ToPtr());
  NamedCommands::registerCommand("Shoot Fuel", std::move(m_simpleShoot).ToPtr());
  NamedCommands::registerCommand("Stage Fuel", std::move(m_simpleStageIntake).ToPtr());

  NamedCommands::registerCommand("Simple Auto Shoot", std::move(m_simpleAutoShoot).ToPtr());
  NamedCommands::registerCommand("Simple Auto Stage", std::move(m_simpleAutoStage).ToPtr());
  NamedCommands::registerCommand("Simple Auto Agitate", std::move(m_simpleAutoAgitate).ToPtr());


  frc::Shuffleboard::GetTab("Autonomous").Add(m_autoChooser);

  m_autoChooser.SetDefaultOption("OneCycle", m_oneCycle);
  m_autoChooser.AddOption("Center Auto", m_centerAuto);
  m_autoChooser.AddOption("Left Auto", m_leftAuto);
  m_autoChooser.AddOption("Right Auto", m_rightAuto);
}

void RobotContainer::ConfigureBindings() {
  // Configure your trigger bindings here
   frc2::JoystickButton resetButton (&m_driveController, ControllerConstants::kResetButton);

   resetButton.OnTrue(frc2::cmd::RunOnce([&] {m_driveSubsystem.ZeroHeading();}, {}));
  
 #ifndef TESTBOARD
 frc2::JoystickButton intakeButton(&m_operatorController, ControllerConstants::kIntakeButton);
 frc2::JoystickButton outtakeButton(&m_operatorController, ControllerConstants::kOuttakeButton);
 frc2::JoystickButton lowerArmButton(&m_operatorController, ControllerConstants::kLowerArmButton);
 frc2::JoystickButton raiseArmButton(&m_operatorController, ControllerConstants::kRaiseArmButton);


  // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
 intakeButton.WhileTrue(SimpleIntake{&m_intakeSubsystem}.ToPtr());
 outtakeButton.WhileTrue(SimpleOuttake{&m_intakeSubsystem}.ToPtr());
 #endif //Testboard
 lowerArmButton.WhileTrue(SimpleLowerArm{&m_intakeArmSubsystem}.ToPtr());
 raiseArmButton.WhileTrue(SimpleRaiseArm{&m_intakeArmSubsystem}.ToPtr());

 


  // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

  // Schedule `ExampleMethodCommand` when the Xbox controller's B button is
  // pressed, cancelling on release.
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
    return PathPlannerAuto(m_autoChooser.GetSelected()).ToPtr();
}
