// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/controller/PIDController.h>
#include <frc/PowerDistribution.h>

class WinchSubsystem : public frc2::SubsystemBase {
 public:
  WinchSubsystem(frc::DutyCycleEncoder* encoder, frc::PowerDistribution* pdp);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void SetPosition(double sp);

  double GetPosition();

  void RunWinch(double speed);

  bool InRange();

 private:

  rev::CANSparkMax m_Winch{9, rev::CANSparkMax::MotorType::kBrushless};
  frc::DutyCycleEncoder* m_Encoder;
  frc::PIDController m_Controller{25, 0, 0};
  frc::PowerDistribution* m_PDP;

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
