/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/PIDCommand.h>
#include <subsystems/DriveSubsystem.h>
/*
class DriveRotate
    : public frc2::CommandHelper<frc2::PIDCommand, DriveRotate> {
 public:
  DriveRotate(DriveSubsystem* drive, units::degree_t target);

  bool IsFinished() override;

  private:
    const double kTurnP = 0.5;
    const double kTurnI = 0.0;
    const double kTurnD = 0.0;

};
*/