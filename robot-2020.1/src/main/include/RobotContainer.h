/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/XboxController.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>
#include <units/units.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/AddressableLED.h>

#include "Constants.h"

#include "subsystems/DriveSubsystem.h"
#include "subsystems/Blaster.h"
#include "subsystems/WheelOfFortune.h"
#include "subsystems/GrapplingHook.h"
#include "subsystems/Hopper.h"
#include "subsystems/Intake.h"

#include "commands/AutoFireLaser.h"
#include "commands/AutoAimCommand.h"
#include "commands/ChargeLaser.h"
#include "commands/DechargeLaser.h"
#include "commands/ManualFireLaser.h"
#include "commands/SpinToColor.h"
#include "commands/SpinWheel3.h"
#include "commands/DriveRotate.h"
#include <wpi/math>
#include <commands/DriveCommand.h>
#include <ntcore.h>
#include <networktables/NetworkTable.h>
#include <frc/Joystick.h>

using std::shared_ptr;

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  void TestVision ();


  frc2::Command* GetAutonomousCommand();
  
  // The driver's controller
  //frc::XboxController m_driverController{OIConstants::kDriverControllerPort};
  //frc::XboxController m_codriverController{OIConstants::CoDriver};
  frc::Joystick m_driverController{OIConstants::kDriverControllerPort};
  frc::Joystick m_codriverController{OIConstants::CoDriver};

 private:
  // The robot's subsystems and commands are defined here...
  //////////////////////////////////////////////////////////////////
  // PWM port 0
  static constexpr int kLength = 62; // number of leds in rings
  std::array<frc::AddressableLED::LEDData, kLength> m_ledBuffer;
  // Must be a PWM header, not MXP or DIO
  frc::AddressableLED m_led{0};
  // Create JD
  shared_ptr<NetworkTable> m_vision;
  

  // The robot's subsystems
  DriveSubsystem m_drive;
  Blaster m_blaster;
  WheelOfFortune m_colorWheel;
  Hopper m_hopper;
  GrapplingHook m_grapplingHook;
  Intake m_collect;

  // The robot's commands
  AutoFireLaser m_AutoShoot;
  AutoAimCommand m_AutoAim{&m_drive};
  ChargeLaser m_PowerUpBlaster{&m_blaster};
  DechargeLaser m_PowerDown{&m_blaster};
  ManualFireLaser m_ManualShoot;
  SpinToColor m_GoToColor{&m_colorWheel};
  SpinWheel3 m_Spin3times{&m_colorWheel};
  DriveRotate m_Drive180{&m_drive,units::degree_t{180}};
  frc2::InstantCommand m_StartIntake{[this] { m_collect.StartIntake(); }, {&m_collect}};
  frc2::InstantCommand m_ReverseIntake{[this] { m_collect.IntakeSpeed(-0.5); }, {&m_collect}};
  frc2::InstantCommand m_StopIntake{[this] { m_collect.StopIntake(); }, {&m_collect}};
  frc2::InstantCommand m_RetractIntake{[this] { m_collect.AdjustIntake(0); }, {&m_collect}};
  frc2::InstantCommand m_HalfExtendIntake{[this] { m_collect.AdjustIntake(1); }, {&m_collect}};
  frc2::InstantCommand m_ExtendIntake{[this] { m_collect.AdjustIntake(2); }, {&m_collect}};
  frc2::InstantCommand m_PositionIntakeHalf{[this] { m_collect.ShootingIntakePositioning(); }, {&m_collect}};
  frc2::InstantCommand m_DeployClimber{[this] {m_grapplingHook.Deploy(true); }, {&m_grapplingHook}};
  frc2::InstantCommand m_UndeployClimber{[this] {m_grapplingHook.Deploy(false); }, {&m_grapplingHook}};
  frc2::InstantCommand m_DriveReverse{[this] {m_drive.Drive(units::meters_per_second_t (0),units::meters_per_second_t (-1),units::radians_per_second_t (0), true); }, {&m_drive}};
  frc2::InstantCommand m_RaiseAngle{[this] {m_blaster.AngleChange(true); }, {&m_blaster}};
  frc2::InstantCommand m_LowerAngle{[this] {m_blaster.AngleChange(false); }, {&m_blaster}};
  frc2::InstantCommand m_ZeroHeading{[this] {m_drive.ZeroHeading(); }, {&m_drive}};
  DriveCommand m_DriveCommand;
  //hopper - need new subsystem for this
  frc2::InstantCommand m_HopperStart{[this] {m_hopper.HopperSpeed(1); }, {&m_hopper}};
  frc2::InstantCommand m_HopperReverse{[this] {m_hopper.HopperSpeed(-1); }, {&m_hopper}};
  frc2::InstantCommand m_HopperStop{[this] {m_hopper.HopperSpeed(0); }, {&m_hopper}};
  frc2::InstantCommand m_DriveSlow{[this] {m_drive.ToggleSlow(); }, {&m_drive}};

  //grappling hook
  frc2::InstantCommand m_DeployClimb{[this] {m_grapplingHook.Deploy(true); }, {&m_grapplingHook}};
  //frc2::InstantCommand m_ClimbUp{[this] {m_grapplingHook.GrapplingHookSpeed(0.5); }, {&m_grapplingHook}};
  //frc2::InstantCommand m_ClimbDown{[this] {m_grapplingHook.GrapplingHookSpeed(-0.5); }, {&m_grapplingHook}};
  //frc2::InstantCommand m_EliCool{[this] {m_grapplingHook.GrapplingHookSpeed(0); }, {&m_grapplingHook}};



  //blaster

  frc2::InstantCommand m_InitiationLineSpeed{[this] {m_blaster.BlasterSpeed(0.53, 0.55); }, {&m_blaster}};
  frc2::InstantCommand m_TrenchSpeed{[this] {m_blaster.BlasterSpeed(0.68, 0.70); }, {&m_blaster}};
  frc2::InstantCommand m_LowGoalSpeed{[this] {m_blaster.BlasterSpeed(0.32, 0.30); }, {&m_blaster}};
  frc2::InstantCommand m_StopBlaster{[this] {m_blaster.BlasterSpeed(0, 0); }, {&m_blaster}};
  frc2::SequentialCommandGroup Start {
    m_TrenchSpeed,
    frc2::WaitCommand{units::second_t(3)}
  };
  frc2::SequentialCommandGroup m_fireAll {
    m_PositionIntakeHalf,
    m_PowerUpBlaster,
    frc2::WaitCommand{units::second_t(3)},
    m_HopperStart,
    frc2::WaitCommand{units::second_t(3)},
    m_HopperStop,
    m_PowerDown
  };

  frc2::SequentialCommandGroup m_stopAll {
    m_HopperStop,
    m_PowerDown,
    m_StopIntake
  };

  //intake

  //grappling hook

  //wheel of fortune

  // The chooser for the autonomous routines
  frc::SendableChooser<frc2::Command*> m_chooser;

  void ConfigureButtonBindings();
};
