// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <numbers>

#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/current.h>
#include <frc/geometry/Pose3d.h>
#include <frc/apriltag/AprilTagFields.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <rev/SparkMax.h>
#include <iostream>
#include <rev/config/SparkMaxConfig.h>



//#define TESTBOARD //disable when on actual robot, to seperate when we are working on the oculus vs the actual robot 
/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */
#define BURNSHOOTERSPARKMAX
//#define TESTBOARD
#define BURNMODULESPARKMAX
#define BURNINTAKESPARKMAX
#define BURNINTAKEARMSPARKMAX
#define BURNAGITATORSPARKMAX
#define PRINTDEBUG

namespace OperatorConstants {

inline constexpr int kDriverControllerPort = 0;
inline constexpr int kOperatorControllerPort = 1;

}  // namespace OperatorConstants

namespace RobotConstants {

constexpr double kVoltageCompensationValue = 11.0;

//const units::meter_t kWheelBase =
//    0.4075_m;  // Distance between centers of front and back wheels on robot
const units::meter_t kFrontWheelOffset =
    0.3725_m;  // Distance between center of robot to front wheels on robot
const units::meter_t kRearWheelOffset =
    -0.035_m;  // Distance between center of robot to back wheels on robot
const units::meter_t kIntakeSideWheelWidth =
    0.8125_m; // Distance between centers of left and right wheels on robot
const units::meter_t kCurveSideWheelWidth =
    0.525_m; // Distance between centers of left and right wheels on robot

}

namespace DriveConstants {
// Driving Parameters - Note that these are not the maximum capable speeds of
// the robot, rather the allowed maximum speeds
constexpr units::meters_per_second_t kMaxSpeed = 4.3_mps;
constexpr units::radians_per_second_t kMaxAngularSpeed{2.0 * std::numbers::pi};

constexpr double kDirectionSlewRate = 6.0;   // radians per second
constexpr double kMagnitudeSlewRate = 7.0;   // percent per second (1 = 100%)
constexpr double kRotationalSlewRate = 8.0;  // percent per second (1 = 100%)

// CAN Sparkmax id numbers
constexpr int kFrontLeftDriveMotorPort = 26;
constexpr int kFrontRightDriveMotorPort = 28;
constexpr int kBackLeftDriveMotorPort = 22;
constexpr int kBackRightDriveMotorPort = 20;

constexpr int kFrontLeftTurningMotorPort = 27;
constexpr int kFrontRightTurningMotorPort = 29;
constexpr int kBackLeftTurningMotorPort = 23;
constexpr int kBackRightTurningMotorPort = 21;

// PID Controller for the auto rotation of the robot
constexpr double kRotationP = 2.5;
constexpr double kRotationI = 0.002;
constexpr double kRotationD = 0.2;

// Anolog input ports on roborio
constexpr int kFrontLeftTurningEncoderPort = kFrontLeftTurningMotorPort;
constexpr int kFrontRightTurningEncoderPort = kFrontRightTurningMotorPort;
constexpr int kBackLeftTurningEncoderPort = kBackLeftTurningMotorPort;
constexpr int kBackRightTurningEncoderPort = kBackRightTurningMotorPort;

// Offsets in radians for the encoders. the first number to to make zero forward, after that we
// subtract an additional pi to make the full range -pi to pi instead of 0 to 2pi
//constexpr double kFrontLeftDriveEncoderOffset = (1.9249 - (std::numbers::pi / 2) + (std::numbers::pi / 3)) + 0.033;
//constexpr double kFrontRightDriveEncoderOffset = (3.2676) - (std::numbers::pi / 3) - 0.062; 
//constexpr double kBackLeftDriveEncoderOffset =  (2.0477) - (2.0 * std::numbers::pi / 3) + std::numbers::pi + 0.050; //(0.6988 + (std::numbers::pi / 2)); 
//constexpr double kBackRightDriveEncoderOffset = (3.8439 + (std::numbers::pi / 2)) - 0.019; //(2.0472 + (std::numbers::pi)); 
constexpr double kFrontLeftTurnEncoderOffset = (std::numbers::pi); //-(std::numbers::pi / 2); //2.789 - (std::numbers::pi / 2) - std::numbers::pi;
constexpr double kFrontRightTurnEncoderOffset = (std::numbers::pi); //4.996 - std::numbers::pi; 
constexpr double kBackLeftTurnEncoderOffset = 0; //5.756; 
constexpr double kBackRightTurnEncoderOffset = -((std::numbers::pi) / 2); //4.407 + (std::numbers::pi / 2) - std::numbers::pi;

constexpr auto kDriveBaseRadius = 0.46_m;

}  // namespace DriveConstants

namespace ModuleConstants {
// Through-hole Encoder on Spark MAX frequency-pwm input
// Invert the turning encoder, since the output shaft rotates in the opposite
// direction of the steering motor in the MAXSwerve Module.
constexpr bool kTurningEncoderInverted = true;

// Calculations required for driving motor conversion factors and feed forward
constexpr double kDrivingMotorFreeSpeedRps =
    5676.0 / 60;  // NEO free speed is 5676 RPM
constexpr units::meter_t kWheelDiameter = 0.0953_m;
constexpr units::meter_t kWheelCircumference =
    kWheelDiameter * std::numbers::pi;

// 6.75:1 Gear Ratio for Driving Motors
constexpr double kDrivingMotorReduction = 6.75;
constexpr double kDriveWheelFreeSpeedRps =
    (kDrivingMotorFreeSpeedRps * kWheelCircumference.value()) /
    kDrivingMotorReduction;

constexpr double kDrivingEncoderPositionFactor =
    (kWheelDiameter.value() * std::numbers::pi) /
    kDrivingMotorReduction;  // meters
constexpr double kDrivingEncoderVelocityFactor =
    ((kWheelDiameter.value() * std::numbers::pi) / kDrivingMotorReduction) /
    60.0;  // meters per second

constexpr double kTurningEncoderPositionFactor =
    (2 * std::numbers::pi);  // radians
constexpr double kTurningEncoderVelocityFactor =
    (2 * std::numbers::pi) / 60.0;  // radians per second

constexpr units::radian_t kTurningEncoderPositionPIDMinInput = 0_rad;
constexpr units::radian_t kTurningEncoderPositionPIDMaxInput =
    units::radian_t{kTurningEncoderPositionFactor};

constexpr double kDrivingP = 0.0;
constexpr double kDrivingI = 0.0;
constexpr double kDrivingD = 0.0;
constexpr double kDrivingFF = (4 / kDriveWheelFreeSpeedRps);
constexpr double kDrivingMinOutput = -1;
constexpr double kDrivingMaxOutput = 1;

constexpr double kTurningP = 1.5;
constexpr double kTurningI = 0.0;
constexpr double kTurningD = 0.0; //was originally 0.15
constexpr double kTurningFF = 0;
constexpr double kTurningMinOutput = -1;
constexpr double kTurningMaxOutput = 1;

constexpr rev::spark::SparkMaxConfig::IdleMode kDrivingMotorIdleMode =  rev::spark::SparkMaxConfig::IdleMode::kBrake;
constexpr rev::spark::SparkMaxConfig::IdleMode kTurningMotorIdleMode = rev::spark::SparkMaxConfig::IdleMode::kBrake;

constexpr units::ampere_t kDrivingMotorCurrentLimit = 40_A;
constexpr units::ampere_t kTurningMotorCurrentLimit = 40_A;

constexpr auto kModuleMaxAngularVelocity =  std::numbers::pi * 9_rad_per_s;  // radians per second ?????????
constexpr auto kModuleMaxAngularAcceleration = std::numbers::pi * 20_rad_per_s / 1_s;  // radians per second^2
constexpr auto kModuleMaxLinearVelocity = 4.65_mps;
}  // namespace ModuleConstants

namespace AutoConstants {

// Only Constants here are the PID constants. Look in path planner for max veleocty/acceleration constants
// PID Constants for the tranlation (X and Y movement) of the robot during auto
constexpr double kPTanslationController = 4.0; // 6.0
constexpr double kITanslationController = 0.0; // 1.7
constexpr double kDTanslationController = 0.0; // 0.0

// PID Constants for the rotation, or Yaw of the robot
constexpr double kPRotationController = 5.0; // 5.0
constexpr double kIRotationController = 0.0; // 0.0
constexpr double kDRotationController = 0.0; // 0.0

}  // namespace AutoConstants

namespace ControllerConstants {

// Axis Indexes
constexpr int kDriveLeftYIndex = 1; // An input UP creates a NEGATIVE output
constexpr int kDriveLeftXIndex = 0; // An input RIGHT creates a NEGATIVE output
constexpr int kDriveRightYIndex = 5; // An input UP creates a NEGATIVE output
constexpr int kDriveRightXIndex = 4; // An input RIGHT creates a NEGATIVE output
constexpr int kOperatorLeftTrigger = 2; // Pressing creates a POSITIVE output
constexpr int kOperatorRightTrigger = 3; // Pressing creates a POSITIVE output
constexpr int kOperatorLeftXIndex = 0; // An input RIGHT creates a NEGATIVE output
constexpr int kOperatorRightYIndex = 5; // An input UP creates a NEGATIVE output

//Driver Buttons
constexpr int kResetButton = 2;
constexpr int kFieldRelativeButton = 7;
constexpr int kRobotRelativeButton = 8;



//operator buttons
constexpr int kOuttakeButton = 5; // LB
constexpr int kIntakeButton = 6; // RB
constexpr int kRaiseArmButton = 4; // Y
constexpr int kLowerArmButton = 3; // X

}  // namespace Controller Constants

namespace ShooterConstants {

// Intake Motors
constexpr int kShooterRightMotorID = 10;
constexpr int kShooterLeftMotorID = 12;

constexpr double kTurretPower = -1;

constexpr rev::spark::SparkLowLevel::MotorType kShooterMotorType = rev::spark::SparkLowLevel::MotorType::kBrushless;
constexpr rev::spark::SparkMaxConfig::IdleMode kShooterMotorIdleMode = rev::spark::SparkMaxConfig::IdleMode::kBrake;

constexpr units::ampere_t kShooterMotorCurrentLimit = 40_A;

}  // namespace ShooterConstants

namespace StagerConstants {
    
constexpr int kStagerShooterMotorID = 19;

    
constexpr rev::spark::SparkLowLevel::MotorType kStagerMotorType = rev::spark::SparkLowLevel::MotorType::kBrushless;
constexpr rev::spark::SparkMaxConfig::IdleMode kStagerMotorIdleMode = rev::spark::SparkMaxConfig::IdleMode::kBrake;

constexpr units::ampere_t kStagerMotorCurrentLimit = 40_A;

} // namespace StagerConstants

namespace TurretYawConstants {

constexpr int kTurretYawMotorID = 24;

constexpr rev::spark::SparkLowLevel::MotorType kTurretYawMotorType = rev::spark::SparkLowLevel::MotorType::kBrushless;
constexpr rev::spark::SparkMaxConfig::IdleMode kTurretYawMotorIdleMode = rev::spark::SparkMaxConfig::IdleMode::kBrake;

constexpr units::ampere_t kTurretYawMotorCurrentLimit = 40_A;

constexpr int kYawLimitSwitchPort = 8;

} // namespace TurretYawConstants

namespace TurretPitchConstants {

constexpr int kTurretPitchMotorID = 11;

constexpr rev::spark::SparkLowLevel::MotorType kTurretPitchMotorType = rev::spark::SparkLowLevel::MotorType::kBrushless;
constexpr rev::spark::SparkMaxConfig::IdleMode kTurretPitchMotorIdleMode = rev::spark::SparkMaxConfig::IdleMode::kBrake;

constexpr units::ampere_t kTurretPitchMotorCurrentLimit = 40_A;

constexpr int kPitchLimitSwitchPort = 9;

constexpr double kPitchMax = 7.0;
constexpr double kPitchMin = 0.0;

constexpr double kPitchP = 1.0;
constexpr double kPitchI = 0.0;
constexpr double kPitchD = 0.0;

constexpr int kPitchMinOutput = -1.0;
constexpr int kPitchMaxOutput = 1.0;

constexpr int kFarSetpoint = 6.0;

} // namespace TurretPitchConstants

namespace IntakeConstants {

constexpr int kIntakeFrontMotorID = 16;
constexpr int kIntakeRearMotorID = 17;
constexpr int kIntakeArmMotorID = 18;

constexpr rev::spark::SparkLowLevel::MotorType kIntakeMotorType = rev::spark::SparkLowLevel::MotorType::kBrushless;
constexpr rev::spark::SparkMaxConfig::IdleMode kIntakeMotorIdleMode = rev::spark::SparkMaxConfig::IdleMode::kBrake;

constexpr units::ampere_t kIntakeMotorCurrentLimit = 40_A;

constexpr double kIntakeLowerLimit = 0.386;
constexpr double kIntakeUpperLimit = 0.633;

constexpr double kIntakeArmP = 1.75;
constexpr double kIntakeArmI = 0.0;
constexpr double kIntakeArmD = 0.0;

constexpr double kMaxOutput = 1.0;
constexpr double kMinOutput = -1.0;

} // namespace IntakeConstants

namespace AgitatorConstants {

constexpr int kAgitatorMotorID = 25;

constexpr rev::spark::SparkLowLevel::MotorType kAgitatorMotorType = rev::spark::SparkLowLevel::MotorType::kBrushless;
constexpr rev::spark::SparkMaxConfig::IdleMode kAgitatorMotorIdleMode = rev::spark::SparkMaxConfig::IdleMode::kBrake;

constexpr units::ampere_t kAgitatorMotorCurrentLimit = 40_A;

} // namespace AgitatorConstants

namespace CameraConstants {

constexpr double kYawP = 0.05;
constexpr double kYawI = 0.0;
constexpr double kYawD = 0.01;

constexpr double kPitchP = 0.3;
constexpr double kPitchI = 0.0;
constexpr double kPitchD = 0.0;

constexpr int counterMax = 250;
constexpr int yawCounterMax = 50;

// Min and Max standard deviations for the apriltag detetion 
constexpr double kMinStandardDeviation = 0.2;
constexpr double kMaxStandardDeviation = 3.0;

// Max speed allowed for adding vidion measurments to the robot pose esitmator
constexpr double kMaxEstimationSpeed = 0.25; // mps


inline constexpr std::string_view kCameraName{"Camera1"};
inline constexpr std::string_view kCamera2Name{"Camera2"};
inline const frc::Transform3d kRobotToCam{
    frc::Translation3d{-0.142_m, 0.21_m, 0.047_m},
    frc::Rotation3d{14_deg, 0_rad, 150_deg}};

inline const frc::Transform3d kRobotToCam2{
    frc::Translation3d{-0.31_m, 0.34_m, 0.1_m},
    frc::Rotation3d{14_deg, 0_deg, 180_deg}};
//    frc::Rotation3d{0_rad, -20_deg, 0_rad}};
inline const frc::AprilTagFieldLayout kTagLayout{
   frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::k2026RebuiltAndyMark)}; 

inline const Eigen::Matrix<double, 3, 1> kSingleTagStdDevs{4, 4, 8};
inline const Eigen::Matrix<double, 3, 1> kMultiTagStdDevs{0.5, 0.5, 1};



/**
 * @param distance The raw distance from the apriltag
 * 
 * @return The standard deviation value for the distance
*/
double GetStandardDeviationFromDistance(double distance);

// Pose3d/transformation2d of the camera relative to the robot
// X if forward, Y is Left, Z is up 
namespace FrontCamera {
    const frc::Translation3d kTranlation3d{(units::meter_t)0.250, (units::meter_t)0.0, (units::meter_t)0.2286};
    const frc::Rotation3d kRotation3d{(units::radian_t)0.0, (units::radian_t)std::numbers::pi / 12, (units::radian_t)0.0};
    const frc::Pose3d kPose3d{kTranlation3d, kRotation3d};
    
} // namespace FrontCamera

namespace OakDLiteCamera {
    const frc::Translation3d kTranlation3d{(units::meter_t)-0.250, (units::meter_t)-0.08, (units::meter_t)0.2286};
    const frc::Rotation3d kRotation3d{(units::radian_t)0.0, (units::radian_t)0.0, (units::radian_t)std::numbers::pi};
    const frc::Pose3d kPose3d{kTranlation3d, kRotation3d};
} // namespace OakDLiteCamera

} // namespace CameraConstants