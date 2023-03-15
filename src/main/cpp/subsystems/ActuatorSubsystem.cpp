// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ActuatorSubsystem.h"

ActuatorSubsystem::ActuatorSubsystem() = default;

// This method will be called once per scheduler run
void ActuatorSubsystem::Periodic() {}

void ActuatorSubsystem::Run(double set) {
  m_Linear.Set(set);
}
