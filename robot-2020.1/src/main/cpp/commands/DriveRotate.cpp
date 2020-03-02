/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/DriveRotate.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html

DriveRotate::DriveRotate(DriveSubsystem* drive, units::degree_t target) :
     CommandHelper(frc2::PIDController(kTurnP, kTurnI, kTurnD),
                   // Close loop on heading
                    [drive] { return drive->GetHeading(); },
                    // Set reference to target
                    target.to<double>()*wpi::math::pi/180.0,
                    // Pipe output to turn robot
                    [drive](double output) {drive->Drive(units::meters_per_second_t{0}, 
                                                          units::meters_per_second_t{0}, 
                                                          units::radians_per_second_t{output}, 
                                                          true); },
                    // Require the drive
                    {drive}) {
  // Set the controller to be continuous (because it is an angle controller)
  m_controller.EnableContinuousInput(-wpi::math::pi, wpi::math::pi);
  // Set the controller tolerance - the delta tolerance ensures the robot is
  // stationary at the setpoint before it is considered as having reached the
  // reference
  m_controller.SetTolerance(0.1, 0.1);

  AddRequirements({drive});
}

// Returns true when the command should end.
bool DriveRotate::IsFinished() { return GetController().AtSetpoint(); }
