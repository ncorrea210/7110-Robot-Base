// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ClawSubsystem.h"

ClawSubsystem::ClawSubsystem() : 
m_motor(0) 
{}

// This method will be called once per scheduler run
void ClawSubsystem::Periodic() {}

void ClawSubsystem::Run(double val) {
    m_motor.Set(val);
}
