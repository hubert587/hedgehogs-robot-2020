/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/DriveSubsystem.h"

#include <stdlib.h>
#include <frc/geometry/Rotation2d.h>
#include <units/units.h>

#include "Constants.h"

using namespace DriveConstants;

DriveSubsystem::DriveSubsystem()
    : m_frontLeft{"Fr_left",
                  kFrontLeftDriveMotorPort,
                  kFrontLeftTurningMotorPort,
                  #ifdef USE_RIO_ANALOG_FOR_ENCODERS 
                    kFrontLeftTurningEncoderPort, 
                  #endif
                  kFrontLeftDriveEncoderReversed,
                  kFrontLeftTurningEncoderReversed},

      m_frontRight{"Fr_right",
          kFrontRightDriveMotorPort,       kFrontRightTurningMotorPort,
          #ifdef USE_RIO_ANALOG_FOR_ENCODERS 
            kFrontRightTurningEncoderPort, 
          #endif
          kFrontRightDriveEncoderReversed, kFrontRightTurningEncoderReversed},

      m_rearLeft{"Bk_left",
          kRearLeftDriveMotorPort,       kRearLeftTurningMotorPort,
          #ifdef USE_RIO_ANALOG_FOR_ENCODERS 
            kRearLeftTurningEncoderPort, 
          #endif
          kRearLeftDriveEncoderReversed, kRearLeftTurningEncoderReversed},

      m_rearRight{"Bk_right",
          kRearRightDriveMotorPort,       kRearRightTurningMotorPort,
          #ifdef USE_RIO_ANALOG_FOR_ENCODERS 
            kRearRightTurningEncoderPort, 
          #endif
          kRearRightDriveEncoderReversed, kRearRightTurningEncoderReversed},

      m_odometry{kDriveKinematics,
                 frc::Rotation2d(units::radian_t(GetHeading())),
                 frc::Pose2d()} {
                    
                    slow = false;
                                     
                }

void DriveSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
  m_odometry.Update(frc::Rotation2d(units::radian_t(GetHeading())),
                    m_frontLeft.GetState(), m_rearLeft.GetState(),
                    m_frontRight.GetState(), m_rearRight.GetState());
}



void DriveSubsystem::Drive(units::meters_per_second_t xSpeed,
                           units::meters_per_second_t ySpeed,
                           units::radians_per_second_t rot,
                           bool fieldRelative) {

  //round inputs
/*  double round = 100;
  xSpeed = units::meters_per_second_t{floor(xSpeed.to<double>() * round) / round};
  ySpeed = units::meters_per_second_t{floor(ySpeed.to<double>() * round) / round};
  rot = units::radians_per_second_t{floor(rot.to<double>() * round) / round};
*/

  //correct idle noise
  double idle = .1;
  if(fabs(xSpeed.to<double>()) < idle) xSpeed = units::meters_per_second_t{0};
  if(fabs(ySpeed.to<double>()) < idle) ySpeed = units::meters_per_second_t{0};
  if(fabs(rot.to<double>()) < idle) rot = units::radians_per_second_t{0};

  //keep wheel rotation unchanged when joysticks are idle
  //if(xSpeed.to<double>() == 0 && ySpeed.to<double>() == 0 && rot.to<double>() == 0) return;

  // full speed
  xSpeed = xSpeed * AutoConstants::kMaxSpeed.to<double>();
  ySpeed = ySpeed * AutoConstants::kMaxSpeed.to<double>();
  rot = rot * 8;

  if (slow == true){
    frc::SmartDashboard::PutString("Drive.Gear", "Slow");
    xSpeed = xSpeed/3.0;
    ySpeed = ySpeed/3.0;
    rot = rot / 2.2;
  } else {
    frc::SmartDashboard::PutString("Drive.Gear", "Normal");
  }

  frc::SmartDashboard::PutNumber("Drive.xSpeed", (double)xSpeed);
  frc::SmartDashboard::PutNumber("Drive.ySpeed", (double)ySpeed);
  frc::SmartDashboard::PutNumber("Drive.rot", (double)rot);
  frc::SmartDashboard::PutNumber("Drive.GetHeading", GetHeading());
  frc::SmartDashboard::PutBoolean("field Orient", fieldRelative);

  auto states = kDriveKinematics.ToSwerveModuleStates(
      fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                          xSpeed, ySpeed, rot,
                          -frc::Rotation2d(units::radian_t(GetHeading())))
                          //frc::Rotation2d(units::degree_t(0)))
                    : frc::ChassisSpeeds{xSpeed, ySpeed, rot});

  kDriveKinematics.NormalizeWheelSpeeds(&states, AutoConstants::kMaxSpeed);

  //auto [fl, fr, bl, br] = states;
  auto [fl, bl, fr, br] = states;

  // fix angles
  Rotation2d halfpi{units::radian_t(wpi::math::pi/2.0)};
  fl.angle = (fl.angle - halfpi) * -1.0;
  fr.angle = (fr.angle - halfpi) * -1.0;
  bl.angle = (bl.angle - halfpi) * -1.0;
  br.angle = (br.angle - halfpi) * -1.0;
  
  frc::SmartDashboard::PutNumber("FL speed", fl.speed.to<double>());
  frc::SmartDashboard::PutNumber("FL angle", fl.angle.Degrees().to<double>());
  frc::SmartDashboard::PutNumber("FR speed", fr.speed.to<double>());
  frc::SmartDashboard::PutNumber("FR angle", fr.angle.Degrees().to<double>());
  frc::SmartDashboard::PutNumber("BL speed", bl.speed.to<double>());
  frc::SmartDashboard::PutNumber("BL angle", bl.angle.Degrees().to<double>());
  frc::SmartDashboard::PutNumber("BR speed", br.speed.to<double>());
  frc::SmartDashboard::PutNumber("BR angle", br.angle.Degrees().to<double>());

  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_rearLeft.SetDesiredState(bl);
  m_rearRight.SetDesiredState(br);
}

void DriveSubsystem::SetModuleStates(
    std::array<frc::SwerveModuleState, 4> desiredStates) {
  kDriveKinematics.NormalizeWheelSpeeds(&desiredStates,
                                        AutoConstants::kMaxSpeed);
  m_frontLeft.SetDesiredState(desiredStates[0]);
  m_rearLeft.SetDesiredState(desiredStates[1]);
  m_frontRight.SetDesiredState(desiredStates[2]);
  m_rearRight.SetDesiredState(desiredStates[3]);
}

void DriveSubsystem::ResetEncoders() {
  m_frontLeft.ResetEncoders();
  m_rearLeft.ResetEncoders();
  m_frontRight.ResetEncoders();
  m_rearRight.ResetEncoders();
}

double DriveSubsystem::GetHeading() {
  double degrees = std::remainder(m_NavX.GetAngle(), 360) * (kGyroReversed ? -1. : 1.);
  return (degrees * wpi::math::pi) / 180.0;
}

void DriveSubsystem::ZeroHeading() { m_NavX.ZeroYaw(); }

double DriveSubsystem::GetTurnRate() {
  return m_NavX.GetRate() * (kGyroReversed ? -1. : 1.);
}

frc::Pose2d DriveSubsystem::GetPose() { return m_odometry.GetPose(); }

void DriveSubsystem::ResetOdometry(frc::Pose2d pose) {
  m_odometry.ResetPosition(pose,
                           frc::Rotation2d(units::degree_t(GetHeading())));
}

//Going to make auto aiming system here





