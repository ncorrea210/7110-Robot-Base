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
  if (hb::limeLight::HasTarget()) {
    m_timer.Stop();
    m_timer.Reset();
    if (m_drive->GetRad().value() > std::numbers::pi + 0.1 || m_drive->GetRad().value() < std::numbers::pi - 0.1) {
      m_drive->Drive(0_mps, 0_mps, units::radian_t(std::numbers::pi));
      return;
    }
    double yCalc = m_yController.Calculate(-hb::limeLight::GetY(), 0);
    double xCalc = m_xController.Calculate(hb::limeLight::GetX(), 0);
    if (fabs(hb::limeLight::GetX()) < 1)
    xCalc = 0;
    if(fabs(hb::limeLight::GetY()) < 1)
    yCalc = 0;
    m_drive->Drive(
      units::meters_per_second_t(yCalc),
      units::meters_per_second_t(xCalc),
      units::radians_per_second_t(0),
      false
    );
  } else if (!hb::limeLight::HasTarget()) {
    m_timer.Start();
  }
}

// Called once the command ends or is interrupted.
void ToLLTargetCMD::End(bool interrupted) {}

// Returns true when the command should end.
bool ToLLTargetCMD::IsFinished() {
  if (m_timer.Get() > 0.1_s) return true;
  if (fabs(hb::limeLight::GetX()) < 1 && fabs(hb::limeLight::GetY()) < 1) return true;
  else return false;
}
