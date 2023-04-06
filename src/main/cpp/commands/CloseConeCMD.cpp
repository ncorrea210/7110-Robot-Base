// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/CloseConeCMD.h"

CloseConeCMD::CloseConeCMD(ClampSubsystem* clamp) : m_clamp(clamp) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(clamp);
}

// Called when the command is initially scheduled.
void CloseConeCMD::Initialize() {
  m_timer.Start();
}

// Called repeatedly when this Command is scheduled to run
void CloseConeCMD::Execute() {
  m_clamp->RunClamp(-0.25);
}

// Called once the command ends or is interrupted.
void CloseConeCMD::End(bool interrupted) {}

// Returns true when the command should end.
bool CloseConeCMD::IsFinished() {
  if (m_timer.Get().value() < 0.1)
  return false;
  else if (m_clamp->GetCurrent() > 3)
  return true;
}
