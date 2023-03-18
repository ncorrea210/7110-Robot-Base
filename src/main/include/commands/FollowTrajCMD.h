// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Timer.h>

#include "subsystems/DriveSubsystem.h"
#include "utils/Trajectories.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class FollowTrajCMD
    : public frc2::CommandHelper<frc2::CommandBase, FollowTrajCMD> {
 public:
  FollowTrajCMD(DriveSubsystem* drive, frc::Trajectory traj);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;
 private:
  DriveSubsystem* m_drive;
  frc::Trajectory m_traj;
  frc::Timer m_timer;
};
