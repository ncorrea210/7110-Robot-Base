// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ToLLTargetCMD.h"
#include <numbers>
#include <cmath>
#include <utility>
#include <frc/smartdashboard/SmartDashboard.h> 

ToLLTargetCMD::ToLLTargetCMD(DriveSubsystem* drive) : m_drive(drive) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(drive);
}

// Called when the command is initially scheduled.
void ToLLTargetCMD::Initialize() {
  hb::limeLight::SetLED(hb::limeLight::LEDMode::kOn);
}

// Called repeatedly when this Command is scheduled to run
void ToLLTargetCMD::Execute() {
  if (m_drive->m_gyro.GetRad().value() > (std::numbers::pi - 0.1) && m_drive->m_gyro.GetRad().value() < (std::numbers::pi + 0.1))
  m_drive->Drive(
    0_mps,
    0_mps,
    units::radian_t(std::numbers::pi));
  else if (hb::limeLight::HasTarget()) {
  double yCalc = -m_xController.Calculate(hb::limeLight::GetX(), 0);
  double xCalc = m_yController.Calculate(hb::limeLight::GetY(), 0);
  yCalc = std::clamp(yCalc, -0.4, 0.4);
  xCalc = std::clamp(xCalc, -0.5, 0.5);
  m_drive->Drive(
    units::meters_per_second_t(xCalc),
    units::meters_per_second_t(yCalc),
    units::radian_t(0));
  } 
}

// Called once the command ends or is interrupted.
void ToLLTargetCMD::End(bool interrupted) {}

// Returns true when the command should end.
bool ToLLTargetCMD::IsFinished() {
  return false;
}
