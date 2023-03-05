// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SetFarPositionCMD.h"

SetFarPositionCMD::SetFarPositionCMD(ExtensionSubsystem* Extension, WinchSubsystem* Winch) : m_Extension(Extension), m_Winch(Winch) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(Extension);
  AddRequirements(Winch);
}

// Called when the command is initially scheduled.
void SetFarPositionCMD::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void SetFarPositionCMD::Execute() {
  
  m_Winch->SetPosition(0.1);
  if (m_Winch->GetPosition() == 0.1)
  m_Extension->SetMax();
  else
  m_Extension->RunExtension(0);
}

// Called once the command ends or is interrupted.
void SetFarPositionCMD::End(bool interrupted) {}

// Returns true when the command should end.
bool SetFarPositionCMD::IsFinished() {
  if (m_Extension->SwitchHigh() && m_Winch->GetPosition() == 0.1) {
    printf("Called\n");
    return true;
  } else return false;
}
