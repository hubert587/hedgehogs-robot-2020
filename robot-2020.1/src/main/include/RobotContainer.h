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

#include "Constants.h"

#include "subsystems/DriveSubsystem.h"
#include "subsystems/Blaster.h"
#include "subsystems/WheelOfFortune.h"
#include "subsystems/GrapplingHook.h"
#include "subsystems/Hopper.h"
#include "subsystems/Intake.h"

#include "commands/AutoFireLaser.h"
#include "commands/ChargeLaser.h"
#include "commands/DechargeLaser.h"
#include "commands/ManualFireLaser.h"
#include "commands/SpinToColor.h"
#include "commands/SpinWheel3.h"


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

  frc2::Command* GetAutonomousCommand();

 private:
  // The driver's controller
  frc::XboxController m_driverController{OIConstants::kDriverControllerPort};
  frc::XboxController m_codriverController{OIConstants::CoDriver};

  // The robot's subsystems and commands are defined here...
  //////////////////////////////////////////////////////////////////

  // The robot's subsystems
  DriveSubsystem m_drive;
  Blaster m_blaster;
  WheelOfFortune m_colorWheel;
  Hopper m_hopper;
  GrapplingHook m_grapplingHook;
  Intake m_collect;

  // The robot's commands
  AutoFireLaser m_AutoShoot;
  ChargeLaser m_PowerUp{&m_blaster};
  DechargeLaser m_PowerDown{&m_blaster};
  ManualFireLaser m_ManualShoot;
  SpinToColor m_GoToColor{&m_colorWheel};
  SpinWheel3 m_Spin3times{&m_colorWheel};
  frc2::InstantCommand m_ExtendIntake{[this] { m_collect.StartIntake(); }, {&m_collect}};
  frc2::InstantCommand m_RetractIntake{[this] { m_collect.StopIntake(); }, {&m_collect}};
  frc2::InstantCommand m_ReverseIntake{[this] { m_collect.IntakeSpeed(-0.5); }, {&m_collect}};
  frc2::InstantCommand m_UnreverseIntake{[this] { m_collect.IntakeSpeed(0.5); }, {&m_collect}};

  //hopper - need new subsystem for this
  frc2::InstantCommand m_HopperStart{[this] {m_hopper.HopperSpeed(1); }, {&m_hopper}};
  frc2::InstantCommand m_HopperStop{[this] {m_hopper.HopperSpeed(0); }, {&m_hopper}};

  //blaster
  frc2::SequentialCommandGroup fireAll(
    //m_PowerUp,
    ChargeLaser(&m_baster),
    DechargeLaser(&m_baster)   
    //m_HopperStart, 
    //m_HopperStop 
  );
    

  //intake

  //grappling hook

  //wheel of fortune

  // The chooser for the autonomous routines
  frc::SendableChooser<frc2::Command*> m_chooser;

  void ConfigureButtonBindings();
};
