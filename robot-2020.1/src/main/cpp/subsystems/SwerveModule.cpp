/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/SwerveModule.h"

#include <frc/geometry/Rotation2d.h>
#include <wpi/math>
#include "units/units.h"

#include "Constants.h"

SwerveModule::SwerveModule(std::string modname,
                          int driveMotorChannel, 
                          int turningMotorChannel,
                          #ifdef USE_RIO_ANALOG_FOR_ENCODERS 
                            int turningEncoderPort, 
                          #endif
                          bool driveEncoderReversed,
                          bool turningEncoderReversed)
    : m_name(modname),
      m_driveMotor(driveMotorChannel, rev::CANSparkMaxLowLevel::MotorType::kBrushless),
      m_turningMotor(turningMotorChannel, rev::CANSparkMaxLowLevel::MotorType::kBrushless),
      #ifdef USE_RIO_ANALOG_FOR_ENCODERS 
        m_turnEncoder(turningEncoderPort), 
      #endif
      m_reverseDriveEncoder(driveEncoderReversed),
      m_reverseTurningEncoder(turningEncoderReversed) {

      m_driveMotor.RestoreFactoryDefaults();
      m_driveMotor.SetInverted(m_reverseDriveEncoder);
      m_driveMotor.SetSmartCurrentLimit(50);
      m_driveMotor.SetSecondaryCurrentLimit(80);
      
      //test IDle mode brake
      //m_driveMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
      
      //testing ramp
      //m_driveMotor.SetOpenLoopRampRate(.1);
      //m_driveMotor.SetClosedLoopRampRate(.1);
      
      //Wheel diamter x Pi x inches per meter / position counts per wheel rev
      m_driveEncoder.SetPositionConversionFactor(3.94 * wpi::math::pi * 0.0254 / 5.9858051);
      m_driveEncoder.SetVelocityConversionFactor(3.94 * wpi::math::pi * 0.0254 / 5.9858051 / 60);       
      m_driveEncoder.SetPosition(0);

      m_drivePIDController.SetP(driveP);
      m_drivePIDController.SetI(driveI);  
      m_drivePIDController.SetD(driveD);
      m_drivePIDController.SetIZone(driveIz);  
      m_drivePIDController.SetFF(driveFF); 
      m_drivePIDController.SetOutputRange(-1.0, 1.0); 
      // I have crippled the robot

      m_turningMotor.RestoreFactoryDefaults();

#ifndef USE_RIO_ANALOG_FOR_ENCODERS 
      m_turningMotor.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0, 20);
      m_turningMotor.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1, 10);
      m_turningMotor.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2, 10);      
      m_turnEncoder.SetPositionConversionFactor(1.89);
#endif      
      m_turningPIDController.Reset();
      m_turningPIDController.EnableContinuousInput(-wpi::math::pi, wpi::math::pi);
      m_turningPIDController.SetTolerance(0.1);

      frc::SmartDashboard::PutNumber("TestSwerveAngle", 0);
}

frc::SwerveModuleState SwerveModule::GetState() {
  return {units::meters_per_second_t{m_driveEncoder.GetVelocity()},
          frc::Rotation2d(units::radian_t(m_turnEncoder.GetPosition() - wpi::math::pi))};
}

void SwerveModule::SetDesiredState(frc::SwerveModuleState& state) {

  //if (fabs(state.speed.to<double>()) < 0.05){ return; }
  //double encread = ((m_turnEncoder.GetPosition() - floor(m_turnEncoder.GetPosition())) * 2 * wpi::math::pi) - wpi::math::pi;
  double encread = m_turnEncoder.GetPosition() - wpi::math::pi;
  double newPos = (double)state.angle.Radians();

  //newPos = frc::SmartDashboard::GetNumber("TestSwerveAngle", 0);

  frc::SmartDashboard::PutNumber(m_name + " Enc", encread);
  //frc::SmartDashboard::PutNumber(m_name + " Angle", newPos);
  //frc::SmartDashboard::PutNumber(m_name + " Speed", state.speed.to<double>());
  //should stop wheel from turnforward to back
  /*newPos = WrapAngle(newPos);
  double dist = fabs(newPos - encread);
  if (dist > wpi::math::pi / 2.0 && dist < 3.0 * wpi::math::pi / 2.0) {
      newPos = WrapAngle(newPos + wpi::math::pi);
      state.speed *= -1; 
  }*/

  //reverse everything
  /*
  double dist = fabs(newPos - encread);
  if (dist > wpi::math::pi / 2.0 && dist < 3.0 * wpi::math::pi / 2.0) {
    m_reverseDriveEncoder = !m_reverseDriveEncoder;
    m_reverseTurningEncoder = !m_reverseTurningEncoder;
    m_driveMotor.SetInverted(m_reverseDriveEncoder);
  }
  if (m_reverseTurningEncoder) {
    if(newPos > 0) newPos -= wpi::math::pi;
    else if(newPos <= 0) newPos += wpi::math::pi;
  }
  */

  double output = m_turningPIDController.Calculate(encread, newPos);
  if (output > 1.0) output = 1.0;
  if (output < -1.0) output = -1.0;

  m_turningMotor.Set(output);
  //m_turningMotor.Set(.1);

  m_drivePIDController.SetReference(state.speed.to<double>(), rev::ControlType::kVelocity);

}

double SwerveModule::WrapAngle(double angle) {
    if (angle >= wpi::math::pi) angle -= 2.0 * wpi::math::pi;
    if (angle <= -wpi::math::pi) angle += 2.0 * wpi::math::pi;
    return angle;
}

void SwerveModule::ResetEncoders() {
  m_driveEncoder.SetPosition(0);
}
