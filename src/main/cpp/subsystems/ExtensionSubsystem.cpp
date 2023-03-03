// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ExtensionSubsystem.h"

#include <cmath>

ExtensionSubsystem::ExtensionSubsystem(frc::DutyCycleEncoder* encoder) : m_Encoder(encoder) {}

// This method will be called once per scheduler run
void ExtensionSubsystem::Periodic() {
  // printf("Distance: %5.2f\n", m_Extension.GetDistance());
  printf("ABS Encoder: %5.2f\n", m_Encoder->Get());

}

void ExtensionSubsystem::SetPosition(double sp) {
  double calc = m_Controller.Calculate(std::lround(m_Extension.GetDistance()), sp);
  calc = std::clamp(calc, -0.5, 0.5);
  m_Extension.Set(calc);
}

double ExtensionSubsystem::GetPosition() {
  return m_Extension.GetDistance();
}

void ExtensionSubsystem::RunExtension(double set){
  if (m_PDP.GetCurrent(7) < 8) {
    if (m_Extension.GetDistance() < 150.0 || set < 0.0) {
    m_Extension.Set(set);

    return;
    } else if (m_Extension.GetDistance() < 10 || set > 0.0) {
    m_Extension.Set(set);

    return;
    }
  } else m_Extension.Set(0);

  // printf("Current: %5.2f\n", m_PDP.GetCurrent(7));
}

double ExtensionSubsystem::GetAngle() {
  return m_Encoder->GetDistance();
}