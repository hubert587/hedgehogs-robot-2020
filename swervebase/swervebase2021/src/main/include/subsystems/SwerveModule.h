/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/Encoder.h>
#include <frc/Spark.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <wpi/math>
#include <rev/CANSparkMax.h>
#include "Constants.h"
#include "AbsoluteEncoder.h"
#include <ctre/Phoenix.h>

class SwerveModule {
  using radians_per_second_squared_t =
      units::compound_unit<units::radians,
                           units::inverse<units::squared<units::second>>>;

 public:
  SwerveModule(std::string modname, int driveMotorChannel, int turningMotorChannel,
               #ifdef USE_RIO_ANALOG_FOR_ENCODERS 
                    int turningEncoderPort, 
               #endif
               bool driveEncoderReversed, bool turningEncoderReversed);

  frc::SwerveModuleState GetState();

  void SetDesiredState(frc::SwerveModuleState& state);

  void ResetEncoders();

 private:
  // We have to use meters here instead of radians due to the fact that
  // ProfiledPIDController's constraints only take in meters per second and
  // meters per second squared.

  static constexpr units::radians_per_second_t kModuleMaxAngularVelocity =
      units::radians_per_second_t(wpi::math::pi);  // radians per second
  static constexpr units::unit_t<radians_per_second_squared_t>
      kModuleMaxAngularAcceleration =
          units::unit_t<radians_per_second_squared_t>(
              wpi::math::pi * 2.0);  // radians per second squared

  double WrapAngle(double angle);
  
  rev::CANSparkMax m_driveMotor;
  rev::CANSparkMax m_turningMotor;

  rev::CANEncoder m_driveEncoder = m_driveMotor.GetEncoder();

  rev::CANPIDController m_drivePIDController = m_driveMotor.GetPIDController();


  #ifdef USE_RIO_ANALOG_FOR_ENCODERS 
    //AbsoluteEncoder m_turnEncoder; 
     CANCoder m_turnEncoder;
  #else
    rev::CANAnalog m_turnEncoder = m_turningMotor.GetAnalog();
  #endif
  //rev::CANEncoder m_turnEncoder = m_turningMotor.GetEncoder();
  std::string m_name;

  bool m_reverseDriveEncoder;
  bool m_reverseTurningEncoder;

  double turnP = -0.3;
  double turnI = 0.0001;
  double turnD = 0.00000001;

  double driveP = 0.2;
  double driveI = 0.01;
  double driveD = 0.5;
  double driveIz = 0.001;
  double driveFF = 0.01;

  frc2::PIDController m_turningPIDController{turnP, turnI, turnD};
};
