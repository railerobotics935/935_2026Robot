// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/JoystickButton.h>

#include "commands/Autos.h"
#include "commands/ExampleCommand.h"

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureBindings();

  // Set Default Commands for Subsystems
  //m_driveSubsystem.SetDefaultCommand(std::move(m_driveWithController));
  #ifndef TESTBOARD
  m_turretPitchSubsystem.SetDefaultCommand(std::move(m_simpleMoveTurretPitch));
  m_turretYawSubsystem.SetDefaultCommand(std::move(m_simpleRotateTurretYaw));
  #endif //testboard
}

void RobotContainer::ConfigureBindings() {
  // Configure your trigger bindings here
  
 #ifndef TESTBOARD
 frc2::JoystickButton shootButton (&m_operatorController, ControllerConstants::kShootButton);
 frc2::JoystickButton stagerIntakeButton (&m_operatorController, ControllerConstants::kStagerIntakeButton);
 frc2::JoystickButton intakeButton (&m_operatorController, ControllerConstants::kIntakeButton);
 frc2::JoystickButton outtakeButton (&m_operatorController, ControllerConstants::kOuttakeButton);

  // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
 shootButton.WhileTrue(SimpleShoot{&m_shooterSubsystem}.ToPtr());
 stagerIntakeButton.WhileTrue(SimpleStagerIntake{&m_stagerSubsystem}.ToPtr());
 intakeButton.WhileTrue(SimpleIntake{&m_intakeSubsystem}.ToPtr());
 outtakeButton.WhileTrue(SimpleOuttake{&m_intakeSubsystem}.ToPtr());
 #endif //Testboard
 


  // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

  // Schedule `ExampleMethodCommand` when the Xbox controller's B button is
  // pressed, cancelling on release.
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
}
