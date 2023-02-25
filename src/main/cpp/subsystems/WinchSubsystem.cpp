// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/WinchSubsystem.h"

WinchSubsystem::WinchSubsystem(frc::DutyCycleEncoder* encoder) : m_Encoder(encoder) {}

// This method will be called once per scheduler run
void WinchSubsystem::Periodic() {}

void WinchSubsystem::SetPosition(double sp) {
  if(m_Encoder->GetDistance() > sp)
    m_Winch.Set(-1.0);
  else if (m_Encoder->GetDistance() < sp)
    m_Winch.Set(1.0);
  else return;
}

double WinchSubsystem::GetPosition() {
  return m_Encoder->GetDistance();
}

void WinchSubsystem::RunWinch(double speed) {
  if(m_Encoder->GetDistance() < WINCH_MAX && m_Encoder->GetDistance() > WINCH_MIN) 
    m_Winch.Set(speed);
  else return;
}