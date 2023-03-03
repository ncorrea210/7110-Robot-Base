// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/OpenClawCMD.h"

OpenClawCMD::OpenClawCMD(ClampSubsystem* Clamp) : m_Clamp(Clamp) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(Clamp);
}

// Called when the command is initially scheduled.
void OpenClawCMD::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void OpenClawCMD::Execute() {
  if (i < 10) {
    m_Clamp->RunClaw(0.1);
    i++;
    return;
  } else if (i < 25) {
    m_Clamp->RunClaw(0.25); 
    return; } 
  else{
    m_Clamp->RunClaw(0.5);
  }
}

// Called once the command ends or is interrupted.
void OpenClawCMD::End(bool interrupted) {}

// Returns true when the command should end.
bool OpenClawCMD::IsFinished() {
  if (m_Clamp->GetCurrent() > 3.75) {
    m_Clamp->RunClaw(0);
    return true;
  } else return false;
}
