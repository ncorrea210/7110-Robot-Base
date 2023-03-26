// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/armpositions/InFrameCMD.h"

InFrameCMD::InFrameCMD(ExtensionSubsystem* extension, ActuatorSubsystem* actuator) : m_extension(extension), m_actuator(actuator) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(extension);
  AddRequirements(actuator);
}

// Called when the command is initially scheduled.
void InFrameCMD::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void InFrameCMD::Execute() {
  m_actuator->SetPosition(100);
  if (m_actuator->GetPosition() < 35) return;
  else 
  m_extension->SetMin();
}

// Called once the command ends or is interrupted.
void InFrameCMD::End(bool interrupted) {}

// Returns true when the command should end.
bool InFrameCMD::IsFinished() {
  if (m_extension->GetPosition() == 0 && m_actuator->GetPosition() > 97) return true;
  else return false;
}
