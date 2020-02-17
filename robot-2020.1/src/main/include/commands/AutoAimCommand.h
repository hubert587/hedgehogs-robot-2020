/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/DriveSubsystem.h"

/**
 * An example command that uses an example subsystem.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class AutoAimCommand
    : public frc2::CommandHelper<frc2::CommandBase, AutoAimCommand> {
 public:
  /**
   * Creates a new AutoAimCommand
   *
   * @param driveSubsystem The subsystem used by this command.
   */
  explicit AutoAimCommand(DriveSubsystem* driveSubsystem);

    void Initialize() override;
    
    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;

    void SetTurnAngle(double angle) {m_turnAngle = angle;};

    double GetTurnAngle() {return m_turnAngle;};

 private:
  DriveSubsystem* m_driveSubsystem;

  double m_turnAngle;

  double m_startAngle;

  double m_turnRate;

};
