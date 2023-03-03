// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ExtensionSubsystem.h"

#include <cmath>

ExtensionSubsystem::ExtensionSubsystem(frc::DutyCycleEncoder* encoder, frc::PowerDistribution* pdp) : m_Encoder(encoder), m_PDP(pdp) {}

// This method will be called once per scheduler run
void ExtensionSubsystem::Periodic() {
  // printf("Distance: %5.2f\n", m_Extension.GetDistance());
  printf("ABS Encoder: %5.2f, Extension Position: %5.2f\n", m_Encoder->Get(), m_Extension.GetDistance());
  // printf("Switch: %d\n", (int)m_LimitSwitch.Get());

}

void ExtensionSubsystem::SetPos(double sp) {
  if ((double)m_Encoder->Get() > 0.19) 
  return;
  else if (m_LimitSwitch.Get() && m_Extension.GetDistance() > 50) {
  m_Extension.Set(0);
  return;
  }
  else if (m_LimitSwitch.Get() && m_Extension.GetDistance() < 50) {
  m_Extension.Set(0);
  return;
  }
  else {
  double calc = m_Controller.Calculate(std::lround(m_Extension.GetDistance()), sp);
  calc = std::clamp(calc, -0.5, 0.5);
  m_Extension.Set(calc);
  }
}

double ExtensionSubsystem::GetPosition() {
  return m_Extension.GetDistance();
}

void ExtensionSubsystem::RunExtension(double set){

  m_Extension.Set(set);

  // printf("Current: %5.2f\n", m_PDP.GetCurrent(7));
}

double ExtensionSubsystem::GetAngle() {
  return m_Encoder->GetDistance();
}

void ExtensionSubsystem::ZeroExtension() {
  if (m_LimitSwitch.Get() || m_Extension.GetDistance() > 50)
  m_Extension.Set(-0.5);
  else if (!m_LimitSwitch.Get() && m_Extension.GetDistance() < 50) {
  m_Extension.Set(0);
  m_Extension.SetPosition(0);
  }
}

void ExtensionSubsystem::SetMax() {;
  if(m_LimitSwitch.Get() || m_Extension.GetDistance() < 50) {
    m_Extension.Set(0.7);
  } else if (!m_LimitSwitch.Get() && m_Extension.GetDistance() > 50) {
    m_Extension.Set(0);
    m_Extension.SetPosition(200);
  }
}