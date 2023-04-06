// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/OpenClampCMD.h"

OpenClampCMD::OpenClampCMD(ClampSubsystem* clamp) : m_clamp(clamp) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(clamp);
}

// Called when the command is initially scheduled.
void OpenClampCMD::Initialize() {
  m_timer.Start();
}

// Called repeatedly when this Command is scheduled to run
void OpenClampCMD::Execute() {
  m_clamp->RunClamp(0.25);
}

// Called once the command ends or is interrupted.
void OpenClampCMD::End(bool interrupted) {}

// Returns true when the command should end.
bool OpenClampCMD::IsFinished() {
  if (m_timer.Get().value() < 0.1)
  return false;
  else if (m_clamp->GetCurrent() > 3)
  return true;
  else if (m_timer.Get().value() > 2) {
  m_timer.Reset();
  return true;
  }
  else return false;
}
