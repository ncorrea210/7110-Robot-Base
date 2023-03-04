// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/WinchSubsystem.h"
#include <utility>
#include <cmath>

WinchSubsystem::WinchSubsystem(frc::DutyCycleEncoder* encoder, frc::PowerDistribution* pdp) : m_Encoder(encoder), m_PDP(pdp) {
  m_Winch.SetSmartCurrentLimit(60);
  // m_Winch.BurnFlash();
}

// This method will be called once per scheduler run
void WinchSubsystem::Periodic() {
}

void WinchSubsystem::SetPosition(double sp) {
  double output = m_Controller.Calculate((double)m_Encoder->Get(), sp);
  output = std::clamp(output, -1.0, 1.0);
  m_Winch.Set(output);
}

double WinchSubsystem::GetPosition() {
  return (double)m_Encoder->Get();
}

void WinchSubsystem::RunWinch(double speed) {
  m_Winch.Set(speed);
}