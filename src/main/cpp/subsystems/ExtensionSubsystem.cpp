// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ExtensionSubsystem.h"

#include <cmath>

ExtensionSubsystem::ExtensionSubsystem() {}

void ExtensionSubsystem::Periodic() {
  // if (SwitchLow()) printf("Switch Low\n");
  // if (SwitchHigh()) printf("Switch High\n");
  // printf("LimitSwitch %d\n", (int)m_LimitSwitch.Get());
}

bool ExtensionSubsystem::SwitchHigh() {
  if (m_Extension.GetDistance() > 75 && !m_LimitSwitch.Get()) {
    return true; 
  } else return false;
}

bool ExtensionSubsystem::SwitchLow() {
  if (m_Extension.GetDistance() < 75 && !m_LimitSwitch.Get()) {
    return true; 
  } else return false;
}

void ExtensionSubsystem::SetPos(double sp) {
  double calc = m_Controller.Calculate(std::lround(m_Extension.GetDistance()), sp);
  calc = std::clamp(calc, -0.5, 0.5);

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

double ExtensionSubsystem::GetPosition() {
  return m_Extension.GetDistance();
}

// void ExtensionSubsystem::RunExtension(double set){
//   if (m_Extension.GetDistance() < 50 && !m_LimitSwitch.Get()) {
//     if (set > 0) {
//       m_Extension.Set(set);
//       return;
//     } else {
//       m_Extension.Set(0);
//       return;
//     }
//   }
//   if (m_Extension.GetDistance() > 150 && !m_LimitSwitch.Get()) {
//     if (set < 0) {
//       m_Extension.Set(set);
//       return;
//     } else {
//       m_Extension.Set(0);
//       return;
//     }
//   }
//   m_Extension.Set(set);
// }

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
