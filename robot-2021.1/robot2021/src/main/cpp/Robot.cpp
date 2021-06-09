/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>


void Robot::RobotInit() {

  TargetDetected = false;
  frc::SmartDashboard::PutNumber("Auto,0M,1L,2R", 0);
  frc::SmartDashboard::PutNumber("distmulti", 1.0);
  
  //Vision->GetTable("VisionTarget");//should create table if does not exist
//control loop (where it crashes)

}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() { 
  //frc::SmartDashboard::PutNumber("SearchRed", m_container.Ultrasonic.Get());
  m_container.TestVision();
  frc2::CommandScheduler::GetInstance().Run(); 

  if (Vision /*&& Vision->IsConnected()*/){

    //Distance = Vision->GetNumber("distance", 0);
    //Angle = Vision->GetNumber("targetAngle", 0);
    //TargetDetected = Vision->GetBoolean("targetFound", false); 


  }

}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit() {
  int ver = frc::SmartDashboard::GetNumber("Auto,0M,1L,2R", 0);
  
  if (ver == 1)
    m_autonomousCommand = m_container.GetAutonomousCommandLeft();
  else if (ver == 2)
    m_autonomousCommand = m_container.GetAutonomousCommandRight();
  else if (ver == 3)
    m_autonomousCommand = m_container.GetAutonomousCommandBarrel();
  else if (ver == 4)
    m_autonomousCommand = m_container.GetAutonomousCommandSlalom();
  else if (ver == 5)
    m_autonomousCommand = m_container.GetAutonomousCommandBounce();
  else if (ver == 6)
    m_autonomousCommand = m_container.GetAutonomousCommandSearch();
  else
    m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  // This makes sure that the autonomous stops running when
  // teleop starts running. If you want the autonomous to
  // continue until interrupted by another command, remove
  // this line or comment it out.
  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Cancel();
    m_autonomousCommand = nullptr;
  }
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {

  m_container.TestVision();

}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
