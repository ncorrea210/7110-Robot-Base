// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ExtensionSubsystem.h"

ExtensionSubsystem::ExtensionSubsystem(frc::DutyCycleEncoder* encoder) : m_Encoder(encoder) {}

// This method will be called once per scheduler run
void ExtensionSubsystem::Periodic() {}

void ExtensionSubsystem::SetPosition(double sp) {
  double calc = m_Controller.Calculate(m_Extension.GetDistance(), sp);
  calc = std::clamp(calc, -1.0, 1.0);
  m_Extension.Set(calc);
}

double ExtensionSubsystem::GetPosition() {
  return m_Extension.GetDistance();
}

void ExtensionSubsystem::RunExtension(double set){
  if (E_MIN < m_Extension.GetDistance() < E_MAX) {
    printf("ExtensionDist %5.2f\n", m_Extension.GetDistance());
    m_Extension.Set(set);}
  else m_Extension.Set(0.0);
}

double ExtensionSubsystem::GetAngle() {
  return m_Encoder->GetDistance();
}