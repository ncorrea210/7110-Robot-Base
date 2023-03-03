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
  // printf("Encoder: %5.2f\n", (double)m_Encoder->Get());
  // printf("Current: %5.2f\n", m_PDP->GetCurrent(15));

}

void WinchSubsystem::SetPosition(double sp) {
  // if((double)m_Encoder->Get() > sp)
  //   m_Winch.Set(-0.5);
  // else if ((double)m_Encoder->Get() < sp)
  //   m_Winch.Set(0.5);
  // else return;


  double output = m_Controller.Calculate((double)m_Encoder->Get(), sp);
  output = std::clamp(output, -1.0, 1.0);
  m_Winch.Set(output);
  // printf("output: %5.2f\n", output);

  // double set = (sp - (double)m_Encoder->Get()) * 50;
  // m_Winch.Set(set);

}

double WinchSubsystem::GetPosition() {
  return (double)m_Encoder->Get();
}

void WinchSubsystem::RunWinch(double speed) {
  m_Winch.Set(speed);
  // printf("Encoder: %5.2f\n", m_Encoder->GetDistance());
}