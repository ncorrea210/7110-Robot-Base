// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include <frc/motorcontrol/VictorSP.h>
#include <frc/PowerDistribution.h>

#include <wpi/sendable/SendableBuilder.h>

class ClawSubsystem : public frc2::SubsystemBase {
 public:
  ClawSubsystem(frc::PowerDistribution* pdp);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void Run(double val);

  void InitSendable(wpi::SendableBuilder& builder) override;

 private:

  frc::VictorSP m_motor;

  frc::PowerDistribution* m_pdp;

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
