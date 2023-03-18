// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ExtensionSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <cmath>

#include <cmath>

ExtensionSubsystem::ExtensionSubsystem() {}

void ExtensionSubsystem::Periodic() {
  if (SwitchLow()) m_Extension.SetPosition(0);
  if(SwitchHigh()) m_Extension.SetPosition(158);

}

bool ExtensionSubsystem::SwitchHigh() {
  if (m_Extension.GetDistance() > 50 && !m_LimitSwitch.Get()) {
    return true; 
  } else return false;
}

bool ExtensionSubsystem::SwitchLow() {
  if (m_Extension.GetDistance() < 75 && !m_LimitSwitch.Get()) {
    return true; 
  } else return false;
}

double ExtensionSubsystem::GetPosition() {
  return std::lround(m_Extension.GetDistance());
}

void ExtensionSubsystem::SetPos(double sp) {
  double calc = m_Controller.Calculate(GetPosition(), sp);
  calc = std::clamp(calc, -0.75, 0.75);

  if (SwitchHigh() && calc > 0) {
    m_Extension.Set(0);
    return;
  } else if (SwitchHigh() && calc < 0) {
    m_Extension.Set(calc);
    return;
  } else if (SwitchLow() && calc > 0) {
    m_Extension.Set(calc);
    return;
  } else if (SwitchLow() && calc < 0) {
    m_Extension.Set(0);
    return;
  } else
  m_Extension.Set(calc);
}


void ExtensionSubsystem::RunExtension(double set) {
  if (SwitchLow()) {
    if (set > 0) {
      m_Extension.Set(set);
    } else {
      m_Extension.Set(0);
    }
  } else if (SwitchHigh()) {
    if (set < 0) {
      m_Extension.Set(set);
    } else {
      m_Extension.Set(0);
    }
  } else 
  m_Extension.Set(set);
}

void ExtensionSubsystem::SetMax() {;
  if(!SwitchHigh()) {
    m_Extension.Set(0.7);
  } else if (SwitchHigh()) {
    m_Extension.Set(0);
    m_Extension.SetPosition(158);
  }
}

void ExtensionSubsystem::SetMin() {
  if (!SwitchLow()) {
    m_Extension.Set(-0.7);
  } else if (SwitchLow()) {
    m_Extension.Set(0);
    m_Extension.SetPosition(0);
  }
}