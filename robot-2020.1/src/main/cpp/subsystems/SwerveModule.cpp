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

SwerveModule::SwerveModule(int driveMotorChannel, int turningMotorChannel,
                           bool driveEncoderReversed,
                           bool turningEncoderReversed)
    : m_driveMotor(driveMotorChannel, rev::CANSparkMaxLowLevel::MotorType::kBrushless),
      m_turningMotor(turningMotorChannel, rev::CANSparkMaxLowLevel::MotorType::kBrushless),
      m_reverseDriveEncoder(driveEncoderReversed),
      m_reverseTurningEncoder(turningEncoderReversed) {

      m_driveMotor.RestoreFactoryDefaults();
      
      //Wheel diamter x Pi x inches per meter / position counts per wheel rev
      m_driveEncoder.SetPositionConversionFactor(3.94 * wpi::math::pi * 0.0254 / 5.9858051);
      m_driveEncoder.SetVelocityConversionFactor(3.94 * wpi::math::pi * 0.0254 / 5.9858051 / 60);       
      m_driveEncoder.SetPosition(0);

      m_drivePIDController.SetP(driveP);
      m_drivePIDController.SetI(driveI);  
      m_drivePIDController.SetD(driveD);
      m_drivePIDController.SetIZone(driveIz);  
      m_drivePIDController.SetFF(driveFF); 
      m_drivePIDController.SetOutputRange(-0.25, 0.25); 
      // I have crippled the robot

      m_turningMotor.RestoreFactoryDefaults();
      m_turningMotor.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0, 20);
      m_turningMotor.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1, 10);
      m_turningMotor.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2, 10);
      
      m_turnEncoder.SetPositionConversionFactor(1.89);
      
      m_turningPIDController.Reset();
      m_turningPIDController.EnableContinuousInput(-1 * wpi::math::pi, wpi::math::pi);
      m_turningPIDController.SetTolerance(0.005);
}

frc::SwerveModuleState SwerveModule::GetState() {
  return {units::meters_per_second_t{m_driveEncoder.GetVelocity()},
          frc::Rotation2d(units::radian_t(m_turnEncoder.GetPosition() - wpi::math::pi / 2))};
}

void SwerveModule::SetDesiredState(frc::SwerveModuleState& state) {

  double encread = m_turnEncoder.GetPosition() - wpi::math::pi / 2;
  double newPos = (double)state.angle.Radians();


  double output = m_turningPIDController.Calculate(encread, newPos);
  if (output > 1.0) output = 1.0;
  if (output < -1.0) output = -1.0;

  m_turningMotor.Set(output);

  m_drivePIDController.SetReference(state.speed.to<double>(), rev::ControlType::kVelocity);

}

void SwerveModule::ResetEncoders() {
  m_driveEncoder.SetPosition(0);
}
