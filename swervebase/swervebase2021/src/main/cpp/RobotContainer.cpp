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
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <commands/DriveCommand.h>
#include <frc/DigitalInput.h>
#include <commands/SearchControllerCommand.h>
using namespace DriveConstants;

//extern double g_Distance;
//extern double g_Angle;
//extern double g_Contours;
//extern bool g_TargetDetected;

RobotContainer::RobotContainer():m_DriveCommand{&m_drive, &m_driverController} {
  // Initialize all of your commands and subsystems here


  //NetworkTable::SetClientMode();
  //NetworkTable::SetTeam(587);
  //NetworkTable::SetIPAddress("roborio-587-frc" ); 
  //NetworkTable::SetIPAddress("10.5.87.2");
  //NetworkTable::Initialize();
  //m_vision = NetworkTable::GetTable("VisionTarget");

  //NetworkTableInstance inst = NetworkTableInstance::GetDefault();
  //NetworkTable table = inst.getTable("datatable");
  
  //auto table = inst.GetTable("datatable");
  //xEntry = table->GetEntry("X");

  for (int i = 0; i < kLength; i++) {
    //Set the value
    m_ledBuffer[i].SetRGB(0,255,0);
  }
  m_led.SetLength(kLength);
  m_led.SetData(m_ledBuffer);
  m_led.Start();


  // Configure the button bindings
  ConfigureButtonBindings();
   
  // Set up default drive command
  
  //m_drive.SetDefaultCommand(m_DriveCommand);

  m_drive.SetDefaultCommand(frc2::RunCommand(
      [this] {
        double x = m_driverController.GetRawAxis(0);
        double y = -m_driverController.GetRawAxis(1);
        double r = -m_driverController.GetRawAxis(2); //4 for new logitech
        bool pressed = false;
        double speed = 0.2;
        if(m_vision) {
          pressed = m_driverController.GetRawButton(7);
          
          if (m_Angle > 0) {
            speed = -0.2;
          }
          frc::SmartDashboard::PutNumber("distance", m_Distance);
          frc::SmartDashboard::PutNumber("targetAngle", m_Angle);
          frc::SmartDashboard::PutNumber("numContours", m_Contours);
          frc::SmartDashboard::PutBoolean("targetFound", m_TargetDetected);
          frc::SmartDashboard::PutBoolean("THE AIM BUTTON", pressed);
          frc::SmartDashboard::PutNumber("Aim Speed", speed);
        } else {

        }
        m_drive.Drive(units::meters_per_second_t(x),
                      units::meters_per_second_t(y),
                      units::radians_per_second_t((pressed && fabs(m_Angle) > .03) ? speed : r),
                      
                      true);
                      //false);
        //frc::SmartDashboard::PutNumber("x_axis", x);
        //frc::SmartDashboard::PutNumber("y_axis", y);
        //frc::SmartDashboard::PutNumber("z_axis", r);     
      },
      {&m_drive}));

    /*m_hopper.SetDefaultCommand(frc2::RunCommand (
        [this] {
            m_hopper.AutoHopper();
        },   
    {&m_hopper}));


    m_collect.SetDefaultCommand(frc2::RunCommand (
        [this] {
            m_collect.IntakeSpeed(m_codriverController.GetRawAxis(3));
        },   
    {&m_collect}));

    m_grapplingHook.SetDefaultCommand(frc2::RunCommand (
      [this] {
        m_grapplingHook.Execute();
        },
    {&m_grapplingHook}));*/
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

    //Drive Controller Button Mapping
    //frc2::Button{[&] {return m_driverController.GetRawButton(1);}}.WhenPressed(&m_RaiseAngle);
    //frc2::Button{[&] {return m_driverController.GetRawButton(2);}}.WhenPressed(&m_LowerAngle);
    frc2::Button{[&] {return m_driverController.GetRawButton(5);}}.WhenPressed(&m_DriveSlow).WhenReleased(&m_DriveSlow);
    //frc2::Button{[&] {return m_driverController.GetRawButton(7);}}.WhenPressed(&m_AutoAim);
    //frc2::Button{[&] {return m_driverController.GetRawButton(8);}}.WhenPressed(&m_fireAll);
    //frc2::Button{[&] {return m_driverController.GetRawButton(9);}}.WhenPressed(&m_stopAll);
    frc2::Button{[&] {return m_driverController.GetRawButton(10);}}.WhenPressed(&m_ZeroHeading);
    //frc2::Button{[&] {return m_codriverController.GetRawButton(1);}}.WhenPressed(&m_PowerUpBlaster); 
    //frc2::Button{[&] {return m_driverController.GetRawButton(5);}}.WhenPressed(&m_PowerDown); 
    //frc2::Button{[&] {return m_driverController.GetRawButton(2);}}.WhenPressed(&m_Drive180);

    //frc2::Button{[&] {return m_codriverController.GetRawButton(8);}}.WhenPressed(&m_ManualShoot);  
    //frc2::Button{[&] {return m_codriverController.GetRawButton(4);}}.WhenPressed(&m_GoToColor);
    //frc2::Button{[&] {return m_codriverController.GetRawButton(2);}}.WhenPressed(&m_Spin3times);
    //frc2::Button{[&] {return m_codriverController.GetRawButton(6);}}.WhenPressed(&m_StartIntake);
    //frc2::Button{[&] {return m_codriverController.GetRawButton(4);}}.WhenPressed(&m_ReverseIntake);
    //frc2::Button{[&] {return m_codriverController.GetRawButton(5);}}.WhenPressed(&m_StopIntake);
    //frc2::Button{[&] {return m_codriverController.GetRawButton(9);}}.WhenPressed(&m_ReverseIntake).WhenReleased(&m_StartIntake);
    
    //Codriver Controller Button Mapping
    //frc2::Button{[&] {return m_codriverController.GetRawButton(1);}}.WhenPressed(&m_RetractIntake);
    //frc2::Button{[&] {return m_codriverController.GetRawButton(2);}}.WhenPressed(&m_HalfExtendIntake);
    //frc2::Button{[&] {return m_codriverController.GetRawButton(3);}}.WhenPressed(&m_ExtendIntake);
    //frc2::Button{[&] {return m_codriverController.GetRawButton(5);}}.WhenPressed(&m_StopBlaster);
    //frc2::Button{[&] {return m_codriverController.GetRawButton(6);}}.WhenPressed(&m_TrenchSpeed);
    //frc2::Button{[&] {return m_codriverController.GetRawButton(7);}}.WhenPressed(&m_LowGoalSpeed);
    //frc2::Button{[&] {return m_codriverController.GetRawButton(8);}}.WhenPressed(&m_InitiationLineSpeed);
    //frc2::Button{[&] {return m_codriverController.GetRawButton(9);}}.WhenPressed(&m_stopAll);
    //frc2::Button{[&] {return m_codriverController.GetRawButton(10);}}.WhenHeld(&m_DeployClimb);
    //frc2::Button{[&] {return m_codriverController.GetRawButton(4);}}.WhenPressed(&m_BlueSpeed);
    //frc2::Button{[&] {return m_codriverController.GetRawButton(11);}}.WhenPressed(&m_DeployClimber);
    //frc2::Button{[&] {return m_codriverController.GetRawButton(12);}}.WhenPressed(&m_UndeployClimber);
}


void RobotContainer::TestVision () {

  m_vision = nt::NetworkTableInstance::GetDefault().GetTable("VisionTarget");
  if(m_vision) {
    m_Distance = m_vision->GetNumber("distance", 0);
    m_Angle = m_vision->GetNumber("targetAngle", 0);
    m_Contours = m_vision->GetNumber("numContours", -1);
    m_TargetDetected = m_vision->GetBoolean("targetFound", false);
    frc::SmartDashboard::PutNumber("distance", m_Distance);
    frc::SmartDashboard::PutNumber("targetAngle", m_Angle);
    frc::SmartDashboard::PutNumber("numContours", m_Contours);
    frc::SmartDashboard::PutBoolean("targetFound", m_TargetDetected);
  } else {
    frc::SmartDashboard::PutNumber("distance", -1);
  }
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
      //frc::Pose2d(0_m, 0_m, frc::Rotation2d(180_deg)),
      // Pass through these two interior waypoints, making an 's' curve path
      {frc::Translation2d(-1_m, 1_m), frc::Translation2d(-2_m, 2_m)},
      // End 3 meters straight ahead of where we started, facing forward
      frc::Pose2d(-3_m, 3_m, frc::Rotation2d(0_deg)),
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
    /*frc2::SequentialCommandGroup{
      m_PositionIntakeHalf,
      m_RaiseAngle,
      m_PowerUpBlaster,
      frc2::WaitCommand{units::second_t(3)},
      m_HopperStart,
      frc2::WaitCommand{units::second_t(3)},
      m_HopperStop,
      m_PowerDown,
      //m_Drive180
    },*/
    std::move(swerveControllerCommand), std::move(swerveControllerCommand),
    frc2::InstantCommand(
      [this]() {
        m_drive.Drive(units::meters_per_second_t(0),
        units::meters_per_second_t(0),
        units::radians_per_second_t(0), false);
      },
  {}));
}

frc2::Command* RobotContainer::GetAutonomousCommandLeft() {
  // Set up config for trajectory
  frc::TrajectoryConfig config(AutoConstants::kMaxSpeed,
                               AutoConstants::kMaxAcceleration);
  // Add kinematics to ensure max speed is actually obeyed
  config.SetKinematics(m_drive.kDriveKinematics);

  // An example trajectory to follow.  All units in meters.
  auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
      //frc::Pose2d(0_m, 0_m, frc::Rotation2d(180_deg)),
      // Pass through these two interior waypoints, making an 's' curve path
      {frc::Translation2d(1_m, -1_m), frc::Translation2d(2_m, -3_m)},
      // End 3 meters straight ahead of where we started, facing forward
      frc::Pose2d(3_m, -4_m, frc::Rotation2d(0_deg)),
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
    /*frc2::SequentialCommandGroup{
      m_PositionIntakeHalf,
      m_RaiseAngle,
      m_PowerUpBlaster,
      frc2::WaitCommand{units::second_t(3)},
      m_HopperStart,
      frc2::WaitCommand{units::second_t(3)},
      m_HopperStop,
      m_PowerDown,
      //m_Drive180
    },*/
    std::move(swerveControllerCommand), std::move(swerveControllerCommand),
    frc2::InstantCommand(
      [this]() {
        m_drive.Drive(units::meters_per_second_t(0),
        units::meters_per_second_t(0),
        units::radians_per_second_t(0), false);
      },
  {}));
}

frc2::Command* RobotContainer::GetAutonomousCommandRight() {
  // Set up config for trajectory
  frc::TrajectoryConfig config(AutoConstants::kMaxSpeed,
                               AutoConstants::kMaxAcceleration);
  // Add kinematics to ensure max speed is actually obeyed
  config.SetKinematics(m_drive.kDriveKinematics);

  // An example trajectory to follow.  All units in meters.
  auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
      //frc::Pose2d(0_m, 0_m, frc::Rotation2d(180_deg)),
      // Pass through these two interior waypoints, making an 's' curve path
      {frc::Translation2d(1_m, 0_m), frc::Translation2d(2_m, 0_m)},
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
    /*frc2::SequentialCommandGroup{
      m_PositionIntakeHalf,
      m_RaiseAngle,
      m_PowerUpBlaster,
      frc2::WaitCommand{units::second_t(3)},
      m_HopperStart,
      frc2::WaitCommand{units::second_t(3)},
      m_HopperStop,
      m_PowerDown,
      //m_Drive180
    },*/
    std::move(swerveControllerCommand), std::move(swerveControllerCommand),
    frc2::InstantCommand(
      [this]() {
        m_drive.Drive(units::meters_per_second_t(0),
        units::meters_per_second_t(0),
        units::radians_per_second_t(0), false);
      },
  {}));
}



frc2::Command* RobotContainer::GetAutonomousCommandBarrel() {
  m_drive.ResetEncoders();

  // Set up config for trajectory
  frc::TrajectoryConfig config(AutoConstants::kMaxSpeed,
                               AutoConstants::kMaxAcceleration);
  // Add kinematics to ensure max speed is actually obeyed
  config.SetKinematics(m_drive.kDriveKinematics);

  // An example trajectory to follow.  All units in meters.
  auto exampleTrajectory0 = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d(InchesToMeters(0), InchesToMeters(0), frc::Rotation2d(0_deg)),
      //frc::Pose2d(0_m, 0_m, frc::Rotation2d(180_deg)),
      // Pass through these two interior waypoints, making an 's' curve path
      { 
      frc::Translation2d(InchesToMeters(45), InchesToMeters(0)),
      frc::Translation2d(InchesToMeters(90), InchesToMeters(-10)),
      frc::Translation2d(InchesToMeters(135), InchesToMeters(-20)),
      frc::Translation2d(InchesToMeters(170), InchesToMeters(-20)), //corner
      frc::Translation2d(InchesToMeters(160), InchesToMeters(30)),
     
      },
      // End 3 meters straight ahead of where we started, facing forward
      frc::Pose2d(InchesToMeters(140), InchesToMeters(80), frc::Rotation2d(0_deg)),
      // Pass the config
      config);

  auto exampleTrajectory1 = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d(InchesToMeters(140), InchesToMeters(80), frc::Rotation2d(0_deg)),
      //frc::Pose2d(0_m, 0_m, frc::Rotation2d(180_deg)),
      // Pass through these two interior waypoints, making an 's' curve path
      { 
      //frc::Translation2d(InchesToMeters(140), InchesToMeters(80)),
      frc::Translation2d(InchesToMeters(55), InchesToMeters(80)), //corner
      frc::Translation2d(InchesToMeters(55), InchesToMeters(30)),
     
      },
      // End 3 meters straight ahead of where we started, facing forward
      frc::Pose2d(InchesToMeters(55), InchesToMeters(-50), frc::Rotation2d(0_deg)),
      // Pass the config
      config);

  auto exampleTrajectory2 = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d(InchesToMeters(55), InchesToMeters(-50), frc::Rotation2d(0_deg)),
      //frc::Pose2d(0_m, 0_m, frc::Rotation2d(180_deg)),
      // Pass through these two interior waypoints, making an 's' curve path
      { 
      frc::Translation2d(InchesToMeters(160), InchesToMeters(-30)),
      frc::Translation2d(InchesToMeters(240), InchesToMeters(-20)), //corner
      frc::Translation2d(InchesToMeters(240), InchesToMeters(-70)),

     
      },
      // End 3 meters straight ahead of where we started, facing forward
      frc::Pose2d(InchesToMeters(240), InchesToMeters(-110), frc::Rotation2d(0_deg)),
      // Pass the config
      config);

  auto exampleTrajectory3 = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d(InchesToMeters(240), InchesToMeters(-110), frc::Rotation2d(0_deg)),
      //frc::Pose2d(0_m, 0_m, frc::Rotation2d(180_deg)),
      // Pass through these two interior waypoints, making an 's' curve path
      { 
      frc::Translation2d(InchesToMeters(210), InchesToMeters(-110)),
      frc::Translation2d(InchesToMeters(135), InchesToMeters(-110)), //corner
      frc::Translation2d(InchesToMeters(135), InchesToMeters(-60)),
     
      },
      // End 3 meters straight ahead of where we started, facing forward
      frc::Pose2d(InchesToMeters(135), InchesToMeters(0), frc::Rotation2d(0_deg)),
      // Pass the config
      config);

  auto exampleTrajectory4 = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d(InchesToMeters(135), InchesToMeters(0), frc::Rotation2d(0_deg)),
      //frc::Pose2d(0_m, 0_m, frc::Rotation2d(180_deg)),
      // Pass through these two interior waypoints, making an 's' curve path
      { 
      frc::Translation2d(InchesToMeters(165), InchesToMeters(40)),
      frc::Translation2d(InchesToMeters(200), InchesToMeters(80)), //corner
      frc::Translation2d(InchesToMeters(240), InchesToMeters(80)),
     
      },
      // End 3 meters straight ahead of where we started, facing forward
      frc::Pose2d(InchesToMeters(300), InchesToMeters(80), frc::Rotation2d(0_deg)),
      // Pass the config
      config);

  auto exampleTrajectory5 = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d(InchesToMeters(300), InchesToMeters(80), frc::Rotation2d(0_deg)),
      //frc::Pose2d(0_m, 0_m, frc::Rotation2d(180_deg)),
      // Pass through these two interior waypoints, making an 's' curve path
      { 
      frc::Translation2d(InchesToMeters(300), InchesToMeters(30)),
      frc::Translation2d(InchesToMeters(300), InchesToMeters(-30)), //corner
      frc::Translation2d(InchesToMeters(150), InchesToMeters(-10)),
      frc::Translation2d(InchesToMeters(0), InchesToMeters(-80)),
      
     
      },
      // End 3 meters straight ahead of where we started, facing forward
      frc::Pose2d(InchesToMeters(-110), InchesToMeters(-190), frc::Rotation2d(0_deg)),
      // Pass the config
      config);

  auto exampleTrajectory6 = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d(InchesToMeters(-110), InchesToMeters(-110), frc::Rotation2d(0_deg)),
      //frc::Pose2d(0_m, 0_m, frc::Rotation2d(180_deg)),
      // Pass through these two interior waypoints, making an 's' curve path
      { 
      frc::Translation2d(InchesToMeters(300), InchesToMeters(-30)),
      frc::Translation2d(InchesToMeters(300), InchesToMeters(-30)), //corner
      frc::Translation2d(InchesToMeters(150), InchesToMeters(-10)),
      frc::Translation2d(InchesToMeters(0), InchesToMeters(-50)),
      
     
      },
      // End 3 meters straight ahead of where we started, facing forward
      frc::Pose2d(InchesToMeters(-130), InchesToMeters(-30), frc::Rotation2d(0_deg)),
      // Pass the config
      config);


m_drive.ResetOdometry(exampleTrajectory0.InitialPose());

  frc2::SwerveControllerCommand<4> swerveControllerCommand0(
      exampleTrajectory0, [this]() { return m_drive.GetPose(); },

      m_drive.kDriveKinematics,

      frc2::PIDController(AutoConstants::kPXController, 0, 0),
      frc2::PIDController(AutoConstants::kPYController, 0, 0),
      frc::ProfiledPIDController<units::radians>(
          AutoConstants::kPThetaController, 0, 0,
          AutoConstants::kThetaControllerConstraints),

      [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },

      {&m_drive});


  frc2::SwerveControllerCommand<4> swerveControllerCommand1(
      exampleTrajectory1, [this]() { return m_drive.GetPose(); },

      m_drive.kDriveKinematics,

      frc2::PIDController(AutoConstants::kPXController, 0, 0),
      frc2::PIDController(AutoConstants::kPYController, 0, 0),
      frc::ProfiledPIDController<units::radians>(
          AutoConstants::kPThetaController, 0, 0,
          AutoConstants::kThetaControllerConstraints),

      [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },

      {&m_drive});


      
  frc2::SwerveControllerCommand<4> swerveControllerCommand2(
      exampleTrajectory2, [this]() { return m_drive.GetPose(); },

      m_drive.kDriveKinematics,

      frc2::PIDController(AutoConstants::kPXController, 0, 0),
      frc2::PIDController(AutoConstants::kPYController, 0, 0),
      frc::ProfiledPIDController<units::radians>(
          AutoConstants::kPThetaController, 0, 0,
          AutoConstants::kThetaControllerConstraints),

      [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },

      {&m_drive});


      
  frc2::SwerveControllerCommand<4> swerveControllerCommand3(
      exampleTrajectory3, [this]() { return m_drive.GetPose(); },

      m_drive.kDriveKinematics,

      frc2::PIDController(AutoConstants::kPXController, 0, 0),
      frc2::PIDController(AutoConstants::kPYController, 0, 0),
      frc::ProfiledPIDController<units::radians>(
          AutoConstants::kPThetaController, 0, 0,
          AutoConstants::kThetaControllerConstraints),

      [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },

      {&m_drive});


      
  frc2::SwerveControllerCommand<4> swerveControllerCommand4(
      exampleTrajectory4, [this]() { return m_drive.GetPose(); },

      m_drive.kDriveKinematics,

      frc2::PIDController(AutoConstants::kPXController, 0, 0),
      frc2::PIDController(AutoConstants::kPYController, 0, 0),
      frc::ProfiledPIDController<units::radians>(
          AutoConstants::kPThetaController, 0, 0,
          AutoConstants::kThetaControllerConstraints),

      [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },

      {&m_drive});

  
  frc2::SwerveControllerCommand<4> swerveControllerCommand5(
      exampleTrajectory5, [this]() { return m_drive.GetPose(); },

      m_drive.kDriveKinematics,

      frc2::PIDController(AutoConstants::kPXController, 0, 0),
      frc2::PIDController(AutoConstants::kPYController, 0, 0),
      frc::ProfiledPIDController<units::radians>(
          AutoConstants::kPThetaController, 0, 0,
          AutoConstants::kThetaControllerConstraints),

      [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },

      {&m_drive});

  frc2::SwerveControllerCommand<4> swerveControllerCommand6(
      exampleTrajectory6, [this]() { return m_drive.GetPose(); },

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
    std::move(swerveControllerCommand0),
    std::move(swerveControllerCommand1),
    std::move(swerveControllerCommand2),
    std::move(swerveControllerCommand3),
    std::move(swerveControllerCommand4),
    std::move(swerveControllerCommand5),
    //std::move(swerveControllerCommand6),
    frc2::InstantCommand(
      [this]() {
        m_drive.Drive(units::meters_per_second_t(0),
        units::meters_per_second_t(0),
        units::radians_per_second_t(0), false);
      },
  {}));
}

frc2::Command* RobotContainer::GetAutonomousCommandSlalom() {
  m_drive.ResetEncoders();
  // Set up config for trajectory
  frc::TrajectoryConfig config(AutoConstants::kMaxSpeed,
                               AutoConstants::kMaxAcceleration);
  // Add kinematics to ensure max speed is actually obeyed
  config.SetKinematics(m_drive.kDriveKinematics);

  // An example trajectory to follow.  All units in meters.
  auto exampleTrajectory0 = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d(InchesToMeters(0), InchesToMeters(0), frc::Rotation2d(0_deg)),
      //frc::Pose2d(0_m, 0_m, frc::Rotation2d(180_deg)),
      // Pass through these two interior waypoints, making an 's' curve path
      {
        frc::Translation2d(InchesToMeters(30), InchesToMeters(0)),
        frc::Translation2d(InchesToMeters(50), InchesToMeters(0)),
        frc::Translation2d(InchesToMeters(50), InchesToMeters(-30*1.4))
      },
      // End 3 meters straight ahead of where we started, facing forward
      frc::Pose2d(InchesToMeters(50), InchesToMeters(-60*1.4), frc::Rotation2d(0_deg)),
      // Pass the config
      config);
  std::cout << "traj0\n";
  auto exampleTrajectory1 = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d(InchesToMeters(50), InchesToMeters(-60*1.4), frc::Rotation2d(0_deg)),
      {
        frc::Translation2d(InchesToMeters(90), InchesToMeters(-60*1.4)),
        frc::Translation2d(InchesToMeters(120), InchesToMeters(-60*1.4)),
        frc::Translation2d(InchesToMeters(150), InchesToMeters(-60*1.4)),
        frc::Translation2d(InchesToMeters(180), InchesToMeters(-60*1.4)),
        frc::Translation2d(InchesToMeters(210), InchesToMeters(-60*1.4)),
        frc::Translation2d(InchesToMeters(247), InchesToMeters(-60*1.4)),
        frc::Translation2d(InchesToMeters(247), InchesToMeters(-30*1.4))
      },
      // End 3 meters straight ahead of where we started, facing forward
      frc::Pose2d(InchesToMeters(247), InchesToMeters(30), frc::Rotation2d(0_deg)),
      // Pass the config
      config);
std::cout << "traj1\n";
auto exampleTrajectory2 = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d(InchesToMeters(247), InchesToMeters(30), frc::Rotation2d(0_deg)),
      {
        frc::Translation2d(InchesToMeters(282), InchesToMeters(30)),
        frc::Translation2d(InchesToMeters(267), InchesToMeters(0)),
        frc::Translation2d(InchesToMeters(252), InchesToMeters(-30*1.4)),
        //frc::Translation2d(InchesToMeters(270), InchesToMeters(-45*1.4))
      },
      // End 3 meters straight ahead of where we started, facing forward
      frc::Pose2d(InchesToMeters(240), InchesToMeters(-40*1.4), frc::Rotation2d(0_deg)),
      // Pass the config
      config);
std::cout << "traj2\n";
auto exampleTrajectory3 = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d(InchesToMeters(240), InchesToMeters(-40*1.4), frc::Rotation2d(0_deg)),
      {
        //frc::Translation2d(InchesToMeters(240), InchesToMeters(-60*1.4)),
        frc::Translation2d(InchesToMeters(210), InchesToMeters(-40*1.4)),
        frc::Translation2d(InchesToMeters(175), InchesToMeters(-40*1.4)),
        //frc::Translation2d(InchesToMeters(180), InchesToMeters(-30*1.4)),
        //frc::Translation2d(InchesToMeters(180), InchesToMeters(-10*1.4))
        
      },
      // End 3 meters straight ahead of where we started, facing forward
      frc::Pose2d(InchesToMeters(175), InchesToMeters(20*1.4), frc::Rotation2d(0_deg)),
      // Pass the config
      config);
std::cout << "traj3\n";
auto exampleTrajectory4 = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d(InchesToMeters(175), InchesToMeters(20*1.4), frc::Rotation2d(0_deg)),
      {
        frc::Translation2d(InchesToMeters(175), InchesToMeters(60)),
        //frc::Translation2d(InchesToMeters(180), InchesToMeters(20)),
        //frc::Translation2d(InchesToMeters(150), InchesToMeters(20)),
        //frc::Translation2d(InchesToMeters(120), InchesToMeters(20)),
        frc::Translation2d(InchesToMeters(90), InchesToMeters(60))
      },
      // End 3 meters straight ahead of where we started, facing forward
      frc::Pose2d(InchesToMeters(-14), InchesToMeters(65), frc::Rotation2d(0_deg)),
      // Pass the config
      config);
std::cout << "traj4\n";

auto exampleTrajectory5 = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d(InchesToMeters(-14), InchesToMeters(65), frc::Rotation2d(0_deg)),
      {
        frc::Translation2d(InchesToMeters(-14), InchesToMeters(0)),
        frc::Translation2d(InchesToMeters(-14), InchesToMeters(-30)),
        frc::Translation2d(InchesToMeters(-35), InchesToMeters(-30))
        
      },
      // End 3 meters straight ahead of where we started, facing forward
      frc::Pose2d(InchesToMeters(-75), InchesToMeters(-30), frc::Rotation2d(0_deg)),
      // Pass the config
      config);
  std::cout << "traj5\n";
  frc2::SwerveControllerCommand<4> swerveControllerCommand0(
      exampleTrajectory0, [this]() { return m_drive.GetPose(); },

      m_drive.kDriveKinematics,

      frc2::PIDController(AutoConstants::kPXController, 0, 0),
      frc2::PIDController(AutoConstants::kPYController, 0, 0),
      frc::ProfiledPIDController<units::radians>(
          AutoConstants::kPThetaController, 0, 0,
          AutoConstants::kThetaControllerConstraints),

      [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },

      {&m_drive});
  frc2::SwerveControllerCommand<4> swerveControllerCommand1(
      exampleTrajectory1, [this]() { return m_drive.GetPose(); },

      m_drive.kDriveKinematics,

      frc2::PIDController(AutoConstants::kPXController, 0, 0),
      frc2::PIDController(AutoConstants::kPYController, 0, 0),
      frc::ProfiledPIDController<units::radians>(
          AutoConstants::kPThetaController, 0, 0,
          AutoConstants::kThetaControllerConstraints),

      [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },

      {&m_drive});

  frc2::SwerveControllerCommand<4> swerveControllerCommand2(
      exampleTrajectory2, [this]() { return m_drive.GetPose(); },

      m_drive.kDriveKinematics,

      frc2::PIDController(AutoConstants::kPXController, 0, 0),
      frc2::PIDController(AutoConstants::kPYController, 0, 0),
      frc::ProfiledPIDController<units::radians>(
          AutoConstants::kPThetaController, 0, 0,
          AutoConstants::kThetaControllerConstraints),

      [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },

      {&m_drive});

  frc2::SwerveControllerCommand<4> swerveControllerCommand3(
      exampleTrajectory3, [this]() { return m_drive.GetPose(); },

      m_drive.kDriveKinematics,

      frc2::PIDController(AutoConstants::kPXController, 0, 0),
      frc2::PIDController(AutoConstants::kPYController, 0, 0),
      frc::ProfiledPIDController<units::radians>(
          AutoConstants::kPThetaController, 0, 0,
          AutoConstants::kThetaControllerConstraints),

      [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },

      {&m_drive});

  frc2::SwerveControllerCommand<4> swerveControllerCommand4(
      exampleTrajectory4, [this]() { return m_drive.GetPose(); },

      m_drive.kDriveKinematics,

      frc2::PIDController(AutoConstants::kPXController, 0, 0),
      frc2::PIDController(AutoConstants::kPYController, 0, 0),
      frc::ProfiledPIDController<units::radians>(
          AutoConstants::kPThetaController, 0, 0,
          AutoConstants::kThetaControllerConstraints),

      [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },

      {&m_drive});

  frc2::SwerveControllerCommand<4> swerveControllerCommand5(
      exampleTrajectory5, [this]() { return m_drive.GetPose(); },

      m_drive.kDriveKinematics,

      frc2::PIDController(AutoConstants::kPXController, 0, 0),
      frc2::PIDController(AutoConstants::kPYController, 0, 0),
      frc::ProfiledPIDController<units::radians>(
          AutoConstants::kPThetaController, 0, 0,
          AutoConstants::kThetaControllerConstraints),

      [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },

      {&m_drive});

  m_drive.ResetOdometry(exampleTrajectory0.InitialPose());
  // no auto
  return new frc2::SequentialCommandGroup(
    std::move(swerveControllerCommand0),
    std::move(swerveControllerCommand1),
    std::move(swerveControllerCommand2),
    std::move(swerveControllerCommand3),
    std::move(swerveControllerCommand4),
    std::move(swerveControllerCommand5),
    frc2::InstantCommand(
      [this]() {
        m_drive.Drive(units::meters_per_second_t(0),
        units::meters_per_second_t(0),
        units::radians_per_second_t(0), false);
      },
  {}));
}

frc2::Command* RobotContainer::GetAutonomousCommandBounce() {
  m_drive.ResetEncoders();
  // Set up config for trajectory
  frc::TrajectoryConfig config(AutoConstants::kMaxSpeed,
                               AutoConstants::kMaxAcceleration);
  // Add kinematics to ensure max speed is actually obeyed
  config.SetKinematics(m_drive.kDriveKinematics);

  // An example trajectory to follow.  All units in meters.
  auto exampleTrajectory0 = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d(InchesToMeters(0), InchesToMeters(0), frc::Rotation2d(0_deg)),
      //frc::Pose2d(0_m, 0_m, frc::Rotation2d(180_deg)),
      // Pass through these two interior waypoints, making an 's' curve path
      {
        frc::Translation2d(InchesToMeters(30), InchesToMeters(-8)),
        frc::Translation2d(InchesToMeters(55), InchesToMeters(-16)),
        frc::Translation2d(InchesToMeters(55), InchesToMeters(-45))
      },
      // End 3 meters straight ahead of where we started, facing forward
      frc::Pose2d(InchesToMeters(55), InchesToMeters(-75), frc::Rotation2d(0_deg)),
      // Pass the config
      config);


  auto exampleTrajectory1 = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d(InchesToMeters(55), InchesToMeters(-75), frc::Rotation2d(0_deg)),
      //frc::Pose2d(0_m, 0_m, frc::Rotation2d(180_deg)),
      // Pass through these two interior waypoints, making an 's' curve path
      {
        frc::Translation2d(InchesToMeters(55), InchesToMeters(-41.5)),
        frc::Translation2d(InchesToMeters(55), InchesToMeters(-23)),
        frc::Translation2d(InchesToMeters(74), InchesToMeters(3)),
        frc::Translation2d(InchesToMeters(82), InchesToMeters(29.5)),
        frc::Translation2d(InchesToMeters(90), InchesToMeters(70)),
        frc::Translation2d(InchesToMeters(127), InchesToMeters(70))
      },
      // End 3 meters straight ahead of where we started, facing forward
      frc::Pose2d(InchesToMeters(140), InchesToMeters(70), frc::Rotation2d(0_deg)),
      // Pass the config
      config);


  auto exampleTrajectory2 = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d(InchesToMeters(137), InchesToMeters(70), frc::Rotation2d(0_deg)),
      //frc::Pose2d(0_m, 0_m, frc::Rotation2d(180_deg)),
      // Pass through these two interior waypoints, making an 's' curve path
      {      
        frc::Translation2d(InchesToMeters(137), InchesToMeters(30)),
        frc::Translation2d(InchesToMeters(137), InchesToMeters(0)),
        frc::Translation2d(InchesToMeters(137), InchesToMeters(-30)),
        frc::Translation2d(InchesToMeters(137), InchesToMeters(-85)),
        frc::Translation2d(InchesToMeters(137), InchesToMeters(-30)),
        frc::Translation2d(InchesToMeters(132), InchesToMeters(0)),
        frc::Translation2d(InchesToMeters(132), InchesToMeters(30))
      },
      // End 3 meters straight ahead of where we started, facing forward
      frc::Pose2d(InchesToMeters(132), InchesToMeters(70), frc::Rotation2d(0_deg)),
      // Pass the config
      config);


  auto exampleTrajectory3 = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d(InchesToMeters(132), InchesToMeters(70), frc::Rotation2d(0_deg)),
      //frc::Pose2d(0_m, 0_m, frc::Rotation2d(180_deg)),
      // Pass through these two interior waypoints, making an 's' curve path
      {
        frc::Translation2d(InchesToMeters(172.5), InchesToMeters(70)),
        frc::Translation2d(InchesToMeters(195), InchesToMeters(70)),
        frc::Translation2d(InchesToMeters(210), InchesToMeters(70))
      },
      // End 3 meters straight ahead of where we started, facing forward
      frc::Pose2d(InchesToMeters(215), InchesToMeters(70), frc::Rotation2d(0_deg)),
      // Pass the config
      config);


  auto exampleTrajectory4 = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d(InchesToMeters(215), InchesToMeters(70), frc::Rotation2d(0_deg)),
      //frc::Pose2d(0_m, 0_m, frc::Rotation2d(180_deg)),
      // Pass through these two interior waypoints, making an 's' curve path
      {
        frc::Translation2d(InchesToMeters(215), InchesToMeters(30)),
        frc::Translation2d(InchesToMeters(215), InchesToMeters(0)),
        frc::Translation2d(InchesToMeters(215), InchesToMeters(-30)),
        frc::Translation2d(InchesToMeters(205), InchesToMeters(-90)),
        frc::Translation2d(InchesToMeters(200), InchesToMeters(-30)),
        frc::Translation2d(InchesToMeters(200), InchesToMeters(0)),
        frc::Translation2d(InchesToMeters(240), InchesToMeters(0))
      },
      // End 3 meters straight ahead of where we started, facing forward
      frc::Pose2d(InchesToMeters(265), InchesToMeters(0), frc::Rotation2d(0_deg)),
      // Pass the config
      config);

  frc2::SwerveControllerCommand<4> swerveControllerCommand0(
      exampleTrajectory0, [this]() { return m_drive.GetPose(); },

      m_drive.kDriveKinematics,

      frc2::PIDController(AutoConstants::kPXController, 0, 0),
      frc2::PIDController(AutoConstants::kPYController, 0, 0),
      frc::ProfiledPIDController<units::radians>(
          AutoConstants::kPThetaController, 0, 0,
          AutoConstants::kThetaControllerConstraints),

      [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },

      {&m_drive});

  frc2::SwerveControllerCommand<4> swerveControllerCommand1(
      exampleTrajectory1, [this]() { return m_drive.GetPose(); },

      m_drive.kDriveKinematics,

      frc2::PIDController(AutoConstants::kPXController, 0, 0),
      frc2::PIDController(AutoConstants::kPYController, 0, 0),
      frc::ProfiledPIDController<units::radians>(
          AutoConstants::kPThetaController, 0, 0,
          AutoConstants::kThetaControllerConstraints),

      [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },

      {&m_drive});

  frc2::SwerveControllerCommand<4> swerveControllerCommand2(
      exampleTrajectory2, [this]() { return m_drive.GetPose(); },

      m_drive.kDriveKinematics,

      frc2::PIDController(AutoConstants::kPXController, 0, 0),
      frc2::PIDController(AutoConstants::kPYController, 0, 0),
      frc::ProfiledPIDController<units::radians>(
          AutoConstants::kPThetaController, 0, 0,
          AutoConstants::kThetaControllerConstraints),

      [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },

      {&m_drive});

  frc2::SwerveControllerCommand<4> swerveControllerCommand3(
      exampleTrajectory3, [this]() { return m_drive.GetPose(); },

      m_drive.kDriveKinematics,

      frc2::PIDController(AutoConstants::kPXController, 0, 0),
      frc2::PIDController(AutoConstants::kPYController, 0, 0),
      frc::ProfiledPIDController<units::radians>(
          AutoConstants::kPThetaController, 0, 0,
          AutoConstants::kThetaControllerConstraints),

      [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },

      {&m_drive});

  frc2::SwerveControllerCommand<4> swerveControllerCommand4(
      exampleTrajectory4, [this]() { return m_drive.GetPose(); },

      m_drive.kDriveKinematics,

      frc2::PIDController(AutoConstants::kPXController, 0, 0),
      frc2::PIDController(AutoConstants::kPYController, 0, 0),
      frc::ProfiledPIDController<units::radians>(
          AutoConstants::kPThetaController, 0, 0,
          AutoConstants::kThetaControllerConstraints),

      [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },

      {&m_drive});


  m_drive.ResetOdometry(exampleTrajectory0.InitialPose());
  // no auto
  return new frc2::SequentialCommandGroup(
    std::move(swerveControllerCommand0),
    std::move(swerveControllerCommand1),
    std::move(swerveControllerCommand2),
    std::move(swerveControllerCommand3),
    std::move(swerveControllerCommand4),
    frc2::InstantCommand(
      [this]() {
        m_drive.Drive(units::meters_per_second_t(0),
        units::meters_per_second_t(0),
        units::radians_per_second_t(0), false);
      },
  {}));
}



//*************SEARCH*************

frc2::Command* RobotContainer::GetAutonomousCommandSearch() {
 frc::DigitalInput Ultrasonic{3}; 
  bool Red = Ultrasonic.Get();
  std::cout<<"Is red?"<<Red<<"\n";
  
  // Set up config for trajectory
  frc::TrajectoryConfig config(AutoConstants::kMaxSpeed,
                               AutoConstants::kMaxAcceleration);
  // Add kinematics to ensure max speed is actually obeyed
  config.SetKinematics(m_drive.kDriveKinematics);


  //*****RED STARTING******
  auto RedStartingTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d(InchesToMeters(0), InchesToMeters(0), frc::Rotation2d(0_deg)),
      {
        frc::Translation2d(InchesToMeters(30), InchesToMeters(0))//First Ball
      },
      frc::Pose2d(InchesToMeters(80), InchesToMeters(30), frc::Rotation2d(0_deg)),//Check for second ball
      // Pass the config
      config);
      std::cout<<"RedStarting\n";
//*****RED PATH A pt1******
  auto RedAOneTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d(InchesToMeters(80), InchesToMeters(30), frc::Rotation2d(0_deg)),
      {
        frc::Translation2d(InchesToMeters(120), InchesToMeters(30))//Get 2nd Ball
      },
      frc::Pose2d(InchesToMeters(110), InchesToMeters(-30), frc::Rotation2d(0_deg)),
      // Pass the config
      config);  
      std::cout<<"RedAOne\n";
//*****RED PATH A pt2******
  auto RedATwoTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d(InchesToMeters(110), InchesToMeters(-30), frc::Rotation2d(0_deg)),
      {
        frc::Translation2d(InchesToMeters(110), InchesToMeters(-85))//Get 3rd Ball
      },
      frc::Pose2d(InchesToMeters(330), InchesToMeters(-80), frc::Rotation2d(0_deg)),//Go to End
      // Pass the config
      config);  
      std::cout<<"RedATwo\n";
//*****RED PATH B pt1******
  auto RedBOneTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d(InchesToMeters(80), InchesToMeters(30), frc::Rotation2d(0_deg)),
      {
        frc::Translation2d(InchesToMeters(80), InchesToMeters(70))
      },
      frc::Pose2d(InchesToMeters(120), InchesToMeters(70), frc::Rotation2d(0_deg)),//Get 2nd ball
      // Pass the config
      config);  
      std::cout<<"RedBOne\n";
//*****RED PATH B pt2******
  auto RedBTwoTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d(InchesToMeters(120), InchesToMeters(70), frc::Rotation2d(0_deg)),
      {
        frc::Translation2d(InchesToMeters(130), InchesToMeters(-15))
      },
      frc::Pose2d(InchesToMeters(330), InchesToMeters(-15), frc::Rotation2d(0_deg)),//Get 3rd ball and go to end
      // Pass the config
      config);  
      std::cout<<"RedBTwo\n";

//*****BLUE STARTING******
  auto BlueStartingTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d(InchesToMeters(0), InchesToMeters(0), frc::Rotation2d(0_deg)),
      {
        frc::Translation2d(InchesToMeters(70), InchesToMeters(-10)),
        frc::Translation2d(InchesToMeters(155), InchesToMeters(-35)),//First Ball
        frc::Translation2d(InchesToMeters(150), InchesToMeters(-35))//First Ball
        
      },
      frc::Pose2d(InchesToMeters(150), InchesToMeters(-130), frc::Rotation2d(0_deg)),//Check for second ball
      // Pass the config
      config);
      std::cout<<"BlueStarting\n";
//*****BLUE PATH A pt1******
  auto BlueAOneTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d(InchesToMeters(120), InchesToMeters(-60), frc::Rotation2d(0_deg)),
      {
        frc::Translation2d(InchesToMeters(120), InchesToMeters(-90))
      },
      frc::Pose2d(InchesToMeters(120), InchesToMeters(-90), frc::Rotation2d(0_deg)),//Get second ball
      // Pass the config
      config);  
      std::cout<<"BlueAOne\n";
//*****BLUE PATH A pt2******
  auto BlueATwoTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d(InchesToMeters(120), InchesToMeters(-90), frc::Rotation2d(0_deg)),
      {
        frc::Translation2d(InchesToMeters(120), InchesToMeters(-60))
      },
      frc::Pose2d(InchesToMeters(300), InchesToMeters(-60), frc::Rotation2d(0_deg)),//Get third ball and go to End
      // Pass the config
      config); 
      std::cout<<"BlueATwo\n";
//*****BLUE PATH B pt1******
  auto BlueBOneTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d(InchesToMeters(150), InchesToMeters(-125), frc::Rotation2d(0_deg)),
      {
        frc::Translation2d(InchesToMeters(180), InchesToMeters(-125)),//Get second ball
      },
      frc::Pose2d(InchesToMeters(180), InchesToMeters(-60), frc::Rotation2d(0_deg)),
      // Pass the config
      config);  
      std::cout<<"BlueBOne\n";
//*****BLUE PATH B pt2******
  auto BlueBTwoTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d(InchesToMeters(180), InchesToMeters(-60), frc::Rotation2d(0_deg)),
      {
        frc::Translation2d(InchesToMeters(180), InchesToMeters(-28))
      },
      frc::Pose2d(InchesToMeters(300), InchesToMeters(-28), frc::Rotation2d(0_deg)),//Get 3rd ball and go to end
      // Pass the config
      config); 
      std::cout<<"BlueBTwo\n";

  frc2::SwerveControllerCommand<4> swerveControllerCommand(
      Red? RedStartingTrajectory:BlueStartingTrajectory, [this]() { return m_drive.GetPose(); },

      m_drive.kDriveKinematics,

      frc2::PIDController(AutoConstants::kPXController, 0, 0),
      frc2::PIDController(AutoConstants::kPYController, 0, 0),
      frc::ProfiledPIDController<units::radians>(
          AutoConstants::kPThetaController, 0, 0,
          AutoConstants::kThetaControllerConstraints),

      [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },

      {&m_drive});

  frc2::SearchControllerCommand<4> SearchControllerCommand0(
      Red? RedAOneTrajectory:BlueBOneTrajectory, 
      Red? RedBOneTrajectory:BlueAOneTrajectory, 
      [this]() { return m_drive.GetPose(); },

      m_drive.kDriveKinematics,

      frc2::PIDController(AutoConstants::kPXController, 0, 0),
      frc2::PIDController(AutoConstants::kPYController, 0, 0),
      frc::ProfiledPIDController<units::radians>(
          AutoConstants::kPThetaController, 0, 0,
          AutoConstants::kThetaControllerConstraints),

      [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },

      {&m_drive});

  frc2::SearchControllerCommand<4> SearchControllerCommand1(
      Red? RedATwoTrajectory:BlueBTwoTrajectory, 
      Red? RedBTwoTrajectory:BlueATwoTrajectory, 
      [this]() { return m_drive.GetPose(); },

      m_drive.kDriveKinematics,

      frc2::PIDController(AutoConstants::kPXController, 0, 0),
      frc2::PIDController(AutoConstants::kPYController, 0, 0),
      frc::ProfiledPIDController<units::radians>(
          AutoConstants::kPThetaController, 0, 0,
          AutoConstants::kThetaControllerConstraints),

      [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },

      {&m_drive});

  m_drive.ResetOdometry(Red? RedStartingTrajectory.InitialPose():BlueStartingTrajectory.InitialPose());
  // no auto
  return new frc2::SequentialCommandGroup(
    /*frc2::SequentialCommandGroup{
      m_ExtendIntake,
      m_StartIntake,
      frc2::WaitCommand{units::second_t(0.5)}
    },*/
    std::move(swerveControllerCommand),
    frc2::InstantCommand(
      [this]() {
        m_drive.Drive(units::meters_per_second_t(0),
        units::meters_per_second_t(0),
        units::radians_per_second_t(0), false);
      },
    {}),
    frc2::WaitCommand{units::second_t(0.5)},
    std::move(SearchControllerCommand0),
    std::move(SearchControllerCommand1),
    frc2::InstantCommand(
      [this]() {
        m_drive.Drive(units::meters_per_second_t(0),
        units::meters_per_second_t(0),
        units::radians_per_second_t(0), false);
      },
  {}));
}