// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include <frc/Timer.h>

#include <units/velocity.h>
#include <units/angular_velocity.h>
#include <units/time.h>

#include "subsystems/DriveSubsystem.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class DriveWithTime
    : public frc2::CommandHelper<frc2::CommandBase, DriveWithTime> {
 public:
  DriveWithTime(DriveSubsystem* drive, units::meters_per_second_t x, units::meters_per_second_t y, 
                units::radians_per_second_t rot, units::second_t time, bool fieldRelative);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  DriveSubsystem* m_drive;

  units::meters_per_second_t m_x;
  units::meters_per_second_t m_y;
  units::radians_per_second_t m_rot;

  units::second_t m_time;

  frc::Timer m_timer;

  bool m_fieldRelative;
};
