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

  /**
   * @brief Checks if the limit switch is true
   * 
   * @returns true if Limit Switch is on and arm is high
  */
  bool SwitchHigh();

  /**
   * @brief Checks if the limit switch is true
   * 
   * @returns true if limit switch is on and arm is low
  */
  bool SwitchLow();

  /**
   * @brief Sets the arm position using a PID controller
   * 
   * @param setpoint of the arm
  */
  void SetPos(double sp);

  /**
   * @brief Gets the position of the arm using the Neo 550 encoder
   * 
   * @returns the position of the arm
  */
  double GetPosition();

  /**
   * @brief Runs the arm to the max position
  */
  void SetMax();

  void RunExtension(double speed);

  void SetMin();

 private:
  hb::NeoMotor m_Extension{9, rev::CANSparkMax::MotorType::kBrushless, rev::CANSparkMax::IdleMode::kBrake};
  frc::PIDController m_Controller{0.2125, 0, 0.004125};
  frc::DigitalInput m_LimitSwitch{0};

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
