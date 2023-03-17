// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ActuatorSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <cmath>

ActuatorSubsystem::ActuatorSubsystem() = default;

// This method will be called once per scheduler run
void ActuatorSubsystem::Periodic() {
  frc::SmartDashboard::PutNumber("Lin Pot", GetPosition());
}

double ActuatorSubsystem::GetPosition() const {
  return std::lround(100 * (((100 * m_Pot.Get().value()) - 3) / 45));
}

void ActuatorSubsystem::Run(const double& set) {
  m_Linear.Set(set);
}

void ActuatorSubsystem::SetPosition(const int& position) {
  double calc = m_controller.Calculate(m_Pot.Get().value(), position);
  m_Linear.Set(calc);
}
