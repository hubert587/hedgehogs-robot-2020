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
#include <frc/XboxController.h>
/**
 * An example command that uses an example subsystem.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class DriveCommand
    : public frc2::CommandHelper<frc2::CommandBase, DriveCommand> {
 public:
  /**
   * Creates a new DriveCommand.
   *
   * @param driveSubsystem The subsystem used by this command.
   */
  explicit DriveCommand(DriveSubsystem* driveSubsystem, XboxController* xboxController);

    void Initialize() override;
    
    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;



 private:
  DriveSubsystem* m_driveSubsystem;

  XboxController* m_xboxController;
};


