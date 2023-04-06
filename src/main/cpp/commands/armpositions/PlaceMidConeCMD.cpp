// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/armpositions/PlaceMidConeCMD.h"

PlaceMidConeCMD::PlaceMidConeCMD(ExtensionSubsystem* extension, ActuatorSubsystem* actuator) : m_extension(extension), m_actuator(actuator) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(extension);
  AddRequirements(actuator);
}

// Called when the command is initially scheduled.
void PlaceMidConeCMD::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void PlaceMidConeCMD::Execute() {
  m_extension->SetMax();
  m_actuator->SetPosition(2);
  printf("E Pos: %5.2f, A Pos: %5.2f\n", m_extension->GetPosition(), m_actuator->GetPosition());
}

// Called once the command ends or is interrupted.
void PlaceMidConeCMD::End(bool interrupted) {}

// Returns true when the command should end.
bool PlaceMidConeCMD::IsFinished() {
  if (m_extension->GetPosition() == 158 && m_actuator->GetPosition() <= 4)
  return true;
  else return false;
}
