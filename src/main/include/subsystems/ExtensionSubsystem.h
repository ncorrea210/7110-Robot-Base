// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#define E_MAX 50.0
#define E_MIN 5.0

#include <frc2/command/SubsystemBase.h>
#include <frc/controller/PIDController.h>
#include <frc/DutyCycleEncoder.h>
#include <utility>

#include "utils/NeoMotors.h"

class ExtensionSubsystem : public frc2::SubsystemBase {
 public:
  ExtensionSubsystem(frc::DutyCycleEncoder* encoder);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void SetPosition(double sp);

  double GetPosition();

  void RunExtension(double speed);

  double GetAngle();

 private:
  hb::NeoMotor m_Extension{10, rev::CANSparkMax::MotorType::kBrushless, rev::CANSparkMax::IdleMode::kBrake};
  frc::PIDController m_Controller{0.5, 0, 0};
  frc::DutyCycleEncoder* m_Encoder;
  const double m_Ratio = 1;

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
