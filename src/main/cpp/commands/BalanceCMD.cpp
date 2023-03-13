// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/BalanceCMD.h"
#include <units/velocity.h>
#include <units/angular_velocity.h>
#include <utility>
#include <cmath>

BalanceCMD::BalanceCMD(DriveSubsystem* Drive) : m_Drive(Drive) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(Drive);
}

// Called when the command is initially scheduled.
void BalanceCMD::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void BalanceCMD::Execute() {
  double calc = m_Controller.Calculate(m_Drive->m_gyro.GetRoll(), 0.0);
  m_Drive->Drive(
    units::meters_per_second_t(std::clamp(calc, -0.25, 0.25)),
    0_mps,
    units::radians_per_second_t(0),
    true
  );
}

// Called once the command ends or is interrupted.
void BalanceCMD::End(bool interrupted) {}

// Returns true when the command should end.
bool BalanceCMD::IsFinished() {
  return false;
}
