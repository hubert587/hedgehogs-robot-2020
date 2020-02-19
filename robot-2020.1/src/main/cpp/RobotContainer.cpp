/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "RobotContainer.h"

#include <frc/controller/PIDController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/button/Button.h>
#include <units/units.h>
#include <frc/Joystick.h>
#include "Constants.h"
#include "subsystems/DriveSubsystem.h"

#include <ntcore.h>
#include <networktables/NetworkTable.h>
#include <commands/DriveCommand.h>

using namespace DriveConstants;

RobotContainer::RobotContainer():m_DriveCommand{&m_drive, &m_driverController} {
  // Initialize all of your commands and subsystems here


    NetworkTable::SetClientMode();
    NetworkTable::SetTeam(587);
    NetworkTable::SetIPAddress("roborio-587-frc" ); /*we don't know if this is right */
    NetworkTable::Initialize();
    m_vision = NetworkTable::GetTable("Vision");


  // Configure the button bindings
  ConfigureButtonBindings();

  // Set up default drive command
  
  //m_drive.SetDefaultCommand(m_DriveCommand);
  
  m_drive.SetDefaultCommand(frc2::RunCommand(
      [this] {
        m_drive.Drive(units::meters_per_second_t(
                          m_driverController.GetY(frc::GenericHID::kLeftHand)),
                      units::meters_per_second_t(
                          m_driverController.GetY(frc::GenericHID::kRightHand)),
                      units::radians_per_second_t(
                          m_driverController.GetX(frc::GenericHID::kLeftHand)),
                      true);
        //frc::SmartDashboard::PutNumber("x_axis", m_driverController.GetRawAxis(0));
        //frc::SmartDashboard::PutNumber("y_axis", m_driverController.GetRawAxis(1));
        //frc::SmartDashboard::PutNumber("z_axis", m_driverController.GetRawAxis(2));


        m_drive.Drive(units::meters_per_second_t(     
                            m_driverController.GetRawAxis(0)),
                      units::meters_per_second_t(
                          m_driverController.GetRawAxis(1)),
                      units::radians_per_second_t(
                          m_driverController.GetRawAxis(2)),
                        
                      false);   
     
     
      },
      {&m_drive}));

    m_hopper.SetDefaultCommand(frc2::RunCommand (
        [this] {
            m_hopper.AutoHopper();
        },   
    {&m_hopper}));
    
}



void RobotContainer::ConfigureButtonBindings() {
    /*frc2::JoystickButton(&m_codriverController, 1).WhenPressed(&m_PowerUp); 
    frc2::JoystickButton(&m_codriverController, 3).WhenPressed(&m_PowerDown); 
    frc2::JoystickButton(&m_codriverController, 7).WhenPressed(&m_AutoShoot); 
    frc2::JoystickButton(&m_codriverController, 8).WhenPressed(&m_ManualShoot);  
    frc2::JoystickButton(&m_codriverController, 4).WhenPressed(&m_GoToColor);
    frc2::JoystickButton(&m_codriverController, 2).WhenPressed(&m_Spin3times);
    frc2::JoystickButton(&m_codriverController, 5).WhenPressed(&m_ExtendIntake);
    frc2::JoystickButton(&m_codriverController, 6).WhenPressed(&m_RetractIntake);
    frc2::JoystickButton(&m_codriverController, 9).WhenPressed(&m_ReverseIntake).WhenReleased(&m_StartIntake);
    frc2::JoystickButton(&m_codriverController, 10).WhenPressed(&m_HopperStart).WhenReleased(&m_HopperStop);
    frc2::JoystickButton(&m_codriverController, 11).WhenPressed(&m_DeployClimber);
    frc2::JoystickButton(&m_codriverController, 12).WhenPressed(&m_UndeployClimber);
    */ 


    frc2::Button{[&] {return m_driverController.GetRawButton(8);}}.WhenPressed(&m_LowerAngle);
    frc2::Button{[&] {return m_driverController.GetRawButton(6);}}.WhenPressed(&m_RaiseAngle);
    frc2::Button{[&] {return m_driverController.GetRawButton(7);}}.WhenPressed(&m_fireAll);

    //frc2::Button{[&] {return m_codriverController.GetRawButton(1);}}.WhenPressed(&m_PowerUpBlaster); 
    frc2::Button{[&] {return m_driverController.GetRawButton(5);}}.WhenPressed(&m_PowerDown); 
    //frc2::Button{[&] {return m_codriverController.GetRawButton(8);}}.WhenPressed(&m_ManualShoot);  
    //frc2::Button{[&] {return m_codriverController.GetRawButton(4);}}.WhenPressed(&m_GoToColor);
    //frc2::Button{[&] {return m_codriverController.GetRawButton(2);}}.WhenPressed(&m_Spin3times);
    frc2::Button{[&] {return m_codriverController.GetRawButton(6);}}.WhenPressed(&m_StartIntake);
    frc2::Button{[&] {return m_codriverController.GetRawButton(4);}}.WhenPressed(&m_ReverseIntake);
    frc2::Button{[&] {return m_codriverController.GetRawButton(5);}}.WhenPressed(&m_StopIntake);
    frc2::Button{[&] {return m_codriverController.GetRawButton(1);}}.WhenPressed(&m_RetractIntake);
    frc2::Button{[&] {return m_codriverController.GetRawButton(2);}}.WhenPressed(&m_HalfExtendIntake);
    frc2::Button{[&] {return m_codriverController.GetRawButton(3);}}.WhenPressed(&m_ExtendIntake);
    //frc2::Button{[&] {return m_codriverController.GetRawButton(9);}}.WhenPressed(&m_ReverseIntake).WhenReleased(&m_StartIntake);
    frc2::Button{[&] {return m_codriverController.GetRawButton(7);}}.WhenPressed(&m_HopperStart).WhenReleased(&m_HopperStop);
    frc2::Button{[&] {return m_codriverController.GetRawButton(8);}}.WhenPressed(&m_HopperReverse).WhenReleased(&m_HopperStop);
    //frc2::Button{[&] {return m_codriverController.GetRawButton(11);}}.WhenPressed(&m_DeployClimber);
    //frc2::Button{[&] {return m_codriverController.GetRawButton(12);}}.WhenPressed(&m_UndeployClimber);
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // Set up config for trajectory
  frc::TrajectoryConfig config(AutoConstants::kMaxSpeed,
                               AutoConstants::kMaxAcceleration);
  // Add kinematics to ensure max speed is actually obeyed
  config.SetKinematics(m_drive.kDriveKinematics);

  // An example trajectory to follow.  All units in meters.
  auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
      // Pass through these two interior waypoints, making an 's' curve path
      {frc::Translation2d(1_m, 1_m), frc::Translation2d(2_m, -1_m)},
      // End 3 meters straight ahead of where we started, facing forward
      frc::Pose2d(3_m, 0_m, frc::Rotation2d(0_deg)),
      // Pass the config
      config);

  frc2::SwerveControllerCommand<4> swerveControllerCommand(
      exampleTrajectory, [this]() { return m_drive.GetPose(); },

      m_drive.kDriveKinematics,

      frc2::PIDController(AutoConstants::kPXController, 0, 0),
      frc2::PIDController(AutoConstants::kPYController, 0, 0),
      frc::ProfiledPIDController<units::radians>(
          AutoConstants::kPThetaController, 0, 0,
          AutoConstants::kThetaControllerConstraints),

      [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },

      {&m_drive});

  // no auto
  return new frc2::SequentialCommandGroup(
      std::move(swerveControllerCommand), std::move(swerveControllerCommand),
      frc2::InstantCommand(
          [this]() {
            m_drive.Drive(units::meters_per_second_t(0),
                          units::meters_per_second_t(0),
                          units::radians_per_second_t(0), false);
          },
          {}));
}
