/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/units.h>
#include <wpi/math>
//#include <rev/ColorSensorV3.h> 

#pragma once

#define USE_RIO_ANALOG_FOR_ENCODERS

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or bool constants.  This should not be used for any other purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */
enum canIDs {
    kFrontLeftDriveCanID = 1,
    kFrontLeftTurnCanID = 2,
    kRearLeftDriveCanID = 3,
    kRearLeftTurnCanID = 4,
    kFrontRightDriveCanID = 5,
    kFrontRightTurnCanID = 6,
    kRearRightDriveCanID = 7,
    kRearRightTurnCanID = 8,
    kOuterIntakeMotorCanID = 9,
    kInnerIntakeMotorCanID = 10,
    kHopperMotorCanID = 11,
    kLeftBlasterWheelCanID = 13,
    kRightBlasterWheelCanID = 12,
    kClimberLeftMptorCanID = 15,
    kClimberRightMotorCanID = 16,
    kClimberAdjustmentMotorCanID = 17,
    kColorWheelCanID = 18

};

enum solenoidIDs {
    kInnerIntakeSolenoid = 1,
    kOuterIntakeSolenoid = 2,
    kClimbSolenoid = 3,
    kColorSolenoid = 6,
    kAngleSolenoid1 = 4,
    kAngleSolenoid2 = 5
};

namespace DriveConstants {
constexpr int kFrontLeftDriveMotorPort = kFrontLeftDriveCanID;
constexpr int kRearLeftDriveMotorPort = kRearLeftDriveCanID;
constexpr int kFrontRightDriveMotorPort = kFrontRightDriveCanID;
constexpr int kRearRightDriveMotorPort = kRearRightDriveCanID;

constexpr int kFrontLeftTurningMotorPort = kFrontLeftTurnCanID;
constexpr int kRearLeftTurningMotorPort = kRearLeftTurnCanID;
constexpr int kFrontRightTurningMotorPort = kFrontRightTurnCanID;
constexpr int kRearRightTurningMotorPort = kRearRightTurnCanID;

constexpr int kFrontLeftTurningEncoderPort{0};
constexpr int kRearLeftTurningEncoderPort{1};
constexpr int kFrontRightTurningEncoderPort{2};
constexpr int kRearRightTurningEncoderPort{3};

constexpr bool kFrontLeftTurningEncoderReversed = false;
constexpr bool kRearLeftTurningEncoderReversed = true;
constexpr bool kFrontRightTurningEncoderReversed = false;
constexpr bool kRearRightTurningEncoderReversed = true;

constexpr int kFrontLeftDriveEncoderPorts[2]{0, 1};
constexpr int kRearLeftDriveEncoderPorts[2]{2, 3};
constexpr int kFrontRightDriveEncoderPorts[2]{4, 5};
constexpr int kRearRightDriveEncoderPorts[2]{5, 6};

constexpr bool kFrontLeftDriveEncoderReversed = true;
constexpr bool kRearLeftDriveEncoderReversed = false;
constexpr bool kFrontRightDriveEncoderReversed = false;
constexpr bool kRearRightDriveEncoderReversed = false;

constexpr bool kGyroReversed = false;

// These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
// These characterization values MUST be determined either experimentally or
// theoretically for *your* robot's drive. The RobotPy Characterization
// Toolsuite provides a convenient tool for obtaining these values for your
// robot.
constexpr auto ks = 1_V;
constexpr auto kv = 0.8 * 1_V * 1_s / 1_m;
constexpr auto ka = 0.15 * 1_V * 1_s * 1_s / 1_m;

// Example value only - as above, this must be tuned for your drive!
constexpr double kPFrontLeftVel = 0.5;
constexpr double kPRearLeftVel = 0.5;
constexpr double kPFrontRightVel = 0.5;
constexpr double kPRearRightVel = 0.5;
}  // namespace DriveConstants

namespace ModuleConstants {
constexpr double kDriveEncoderVelocityConversionFactor = 1/3;
constexpr int kEncoderCPR = 1024;
constexpr double kWheelDiameterMeters = .15;
constexpr double kDriveEncoderDistancePerPulse =
    // Assumes the encoders are directly mounted on the wheel shafts
    (kWheelDiameterMeters * wpi::math::pi) / static_cast<double>(kEncoderCPR);

constexpr double kTurningEncoderDistancePerPulse =
    // Assumes the encoders are directly mounted on the wheel shafts
    (wpi::math::pi * 2) / static_cast<double>(kEncoderCPR);

constexpr double kPModuleTurningController = 1;
constexpr double kPModuleDriveController = 1;
}  // namespace ModuleConstants

namespace AutoConstants {
using radians_per_second_squared_t =
    units::compound_unit<units::radians,
                         units::inverse<units::squared<units::second>>>;

constexpr auto kMaxSpeed = units::meters_per_second_t(8.0);//4.5);
constexpr auto kMaxAcceleration = units::meters_per_second_squared_t(4.5);
constexpr auto kMaxAngularSpeed = units::radians_per_second_t(3.142);
constexpr auto kMaxAngularAcceleration =
    units::unit_t<radians_per_second_squared_t>(3.142);

constexpr double kPXController = 0.5;
constexpr double kPYController = 0.5;
constexpr double kPThetaController = 0.5;

//

extern const frc::TrapezoidProfile<units::radians>::Constraints
    kThetaControllerConstraints;

}  // namespace AutoConstants

namespace OIConstants {
constexpr int kDriverControllerPort = 0;
constexpr int CoDriver = 1;
}  // namespace OIConstants



