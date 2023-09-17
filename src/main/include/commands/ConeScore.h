// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include <frc/controller/PIDController.h>

#include <functional>
#include <utility>

#include "subsystems/DriveSubsystem.h"

#include "utils/cams/Limelight.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class ConeScore
    : public frc2::CommandHelper<frc2::CommandBase, ConeScore> {
 public:
  ConeScore(DriveSubsystem* drive, std::function<double()> forwards);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:

  DriveSubsystem* m_drive;

  std::function<double()> m_forwards;

  frc::PIDController m_horizontalController{0.05, 0, 0};

};
