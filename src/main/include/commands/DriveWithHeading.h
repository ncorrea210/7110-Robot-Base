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

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class DriveWithHeading
    : public frc2::CommandHelper<frc2::CommandBase, DriveWithHeading> {
 public:
  DriveWithHeading(DriveSubsystem* drive, std::function<double()> x, std::function<double()> y, units::degree_t heading);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;
 
 private:

  DriveSubsystem* m_drive;
  std::function<double()> m_x;
  std::function<double()> m_y;
  units::degree_t m_heading;


  // frc::ProfiledPIDController<units::degree_t> m_controller{0.05, 0, 0, {DriveConstants::kMaxAngularSpeed.convert<units::degree_t>(), DriveConstants::kMaxAngularAcceleration.convert<units::degrees_per_second_squared_t>()}};
  frc2::PIDController m_controller{0.04, 0, 0};

};
