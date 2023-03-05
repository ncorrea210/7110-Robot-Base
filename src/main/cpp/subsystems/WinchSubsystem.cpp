// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/WinchSubsystem.h"
#include <utility>
#include <cmath>
#include <frc/smartdashboard/SmartDashboard.h>

WinchSubsystem::WinchSubsystem(frc::DutyCycleEncoder* encoder, frc::PowerDistribution* pdp) : m_Encoder(encoder), m_PDP(pdp) {
  m_Winch.SetSmartCurrentLimit(60);
  // m_Winch.BurnFlash();
}

// This method will be called once per scheduler run
void WinchSubsystem::Periodic() {
  frc::SmartDashboard::PutNumber("ABS Encoder", (double)m_Encoder->Get());
}

void WinchSubsystem::SetPosition(double sp) {
  double output = m_Controller.Calculate((double)m_Encoder->Get(), sp);
  output = std::clamp(output, -1.0, 1.0);
  if (sp > 0) {
    if ((double)m_Encoder->Get() < 0.04) {
      m_Winch.Set(0);
    } else {
  m_Winch.Set(output);
  }
  } else
  m_Winch.Set(output);
}

double WinchSubsystem::GetPosition() {
  return (double)m_Encoder->Get();
}

void WinchSubsystem::RunWinch(double speed) {
  if ((double)m_Encoder->Get() < 0.04 && speed < 0)
  m_Winch.Set(0);
  else
  m_Winch.Set(speed);
}