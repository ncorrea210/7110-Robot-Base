// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/controller/PIDController.h>
#include <frc/DigitalInput.h>

#include "utils/NeoMotors.h"

class ExtensionSubsystem : public frc2::SubsystemBase {
 public:
  ExtensionSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  bool SwitchHigh();

  bool SwitchLow();

  void SetPos(double sp);

  double GetPosition();

  void SetMax();

  void RunExtension(double speed);

  void ZeroExtension();

 private:
  hb::NeoMotor m_Extension{9, rev::CANSparkMax::MotorType::kBrushless, rev::CANSparkMax::IdleMode::kBrake};
  frc::PIDController m_Controller{0.2125, 0, 0.004125};
  frc::DigitalInput m_LimitSwitch{0};

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
