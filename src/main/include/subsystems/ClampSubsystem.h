// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "utils/NeoMotors.h"
#include <frc/motorcontrol/VictorSP.h>
#include <frc/PowerDistribution.h>

class ClampSubsystem : public frc2::SubsystemBase {
 public:
  ClampSubsystem(frc::PowerDistribution* pdp);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void RunClamp(double set);

  double GetCurrent();

 private:

  frc::VictorSP m_motor{0};
  frc::PowerDistribution* m_PDP;
  
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};