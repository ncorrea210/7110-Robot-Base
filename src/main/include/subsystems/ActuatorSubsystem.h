// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/motorcontrol/VictorSP.h>
#include <frc/AnalogEncoder.h>
#include <frc/controller/PIDController.h>

class ActuatorSubsystem : public frc2::SubsystemBase {
 public:
  ActuatorSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void Run(const double& set);

  void SetPosition(const double& position);

  double GetPosition() const;

 private:
  frc::VictorSP m_Linear{1};
  frc::AnalogEncoder m_Pot{0}; //No pot for now
  frc::PIDController m_controller {1, 0, 0};

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
