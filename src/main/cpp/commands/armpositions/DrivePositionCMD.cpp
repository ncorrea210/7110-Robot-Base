// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/armpositions/DrivePositionCMD.h"

DrivePositionCMD::DrivePositionCMD(ExtensionSubsystem* extension, ActuatorSubsystem* actuator) : m_extension(extension), m_actuator(actuator) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(extension);
  AddRequirements(actuator);
}

// Called when the command is initially scheduled.
void DrivePositionCMD::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void DrivePositionCMD::Execute() {
  m_extension->SetMin();
  m_actuator->SetPosition(2);
}

// Called once the command ends or is interrupted.
void DrivePositionCMD::End(bool interrupted) {}

// Returns true when the command should end.
bool DrivePositionCMD::IsFinished() {
  if (m_extension->GetPosition() == 0 && m_actuator->GetPosition() <= 4) return true;
  else return false;
}
