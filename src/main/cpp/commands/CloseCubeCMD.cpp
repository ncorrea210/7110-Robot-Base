// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/CloseCubeCMD.h"

CloseCubeCMD::CloseCubeCMD(ClampSubsystem* Clamp) : m_Clamp(Clamp) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(Clamp);
}

// Called when the command is initially scheduled.
void CloseCubeCMD::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void CloseCubeCMD::Execute() {
  if (i = 0) {
    m_Clamp->RunClaw(-0.05);
    i++;
    return;
  } else if (i < 10) {
    m_Clamp->RunClaw(-0.1);
    i++;
    return;
  } else {
    m_Clamp->RunClaw(-0.25);
  }
}

// Called once the command ends or is interrupted.
void CloseCubeCMD::End(bool interrupted) {}

// Returns true when the command should end.
bool CloseCubeCMD::IsFinished() {
    if (m_Clamp->GetCurrent() > 1) {
    m_Clamp->RunClaw(0);
    return true;
  } else return false;
}
