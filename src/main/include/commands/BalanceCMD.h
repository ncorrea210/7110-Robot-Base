// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/controller/PIDController.h>

#include "subsystems/DriveSubsystem.h"


/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class BalanceCMD
    : public frc2::CommandHelper<frc2::CommandBase, BalanceCMD> {
 public:
  BalanceCMD(DriveSubsystem* Drive);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  DriveSubsystem* m_Drive;
  frc::PIDController m_Controller{0.075, 0, 0.002};
};
