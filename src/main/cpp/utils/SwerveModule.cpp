// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "utils/SwerveModule.h"
#include <frc/geometry/Rotation2d.h>
#include <numbers>
#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants.h"

SwerveModule::SwerveModule(int driveMotorChannel, int turningMotorChannel,
                           const int turningEncoderPorts,
                           const double offset)
    : m_driveMotor(driveMotorChannel, rev::CANSparkMax::MotorType::kBrushless, rev::CANSparkMax::IdleMode::kCoast),
      m_turningMotor(turningMotorChannel, rev::CANSparkMax::MotorType::kBrushless, rev::CANSparkMax::IdleMode::kBrake),
      m_turningEncoder(turningEncoderPorts, offset),
      m_id(turningEncoderPorts) {
  // Set the distance per pulse for the drive encoder. We can simply use the
  // distance traveled for one rotation of the wheel divided by the encoder
  // resolution.
  m_driveMotor.SetRatio(
      ModuleConstants::kDriveEncoderDistancePerPulse);

  // Limit the PID Controller's input range between -pi and pi and set the input
  // to be continuous.
  m_turningPIDController.EnableContinuousInput(
      units::radian_t(-std::numbers::pi), units::radian_t(std::numbers::pi));
}

frc::SwerveModuleState SwerveModule::GetState() {
  return {units::meters_per_second_t{m_driveMotor.GetRate()},
          frc::Rotation2d(units::radian_t(m_turningEncoder.Get()))};
}

frc::SwerveModulePosition SwerveModule::GetPosition() {
    return {units::meter_t(m_driveMotor.GetDistance()), units::radian_t(m_turningEncoder.Get())};
}

void SwerveModule::SetDesiredState(
    const frc::SwerveModuleState& referenceState) {

  // m_turningPIDController.SetD(
  // frc::SmartDashboard::GetNumber("DModVal", 0)
  // );

  // Optimize the reference state to avoid spinning further than 90 degrees
  const auto state = frc::SwerveModuleState::Optimize(
      referenceState, units::radian_t(m_turningEncoder.Get()));

  // Calculate the drive output from the drive PID controller.
  
  const auto driveOutput = m_drivePIDController.Calculate(
      -m_driveMotor.GetRate(), state.speed.value());

  // Calculate the turning motor output from the turning PID controller.
  auto turnOutput = m_turningPIDController.Calculate(
      units::radian_t(m_turningEncoder.Get()), state.angle.Radians());


  units::volt_t driveFF = m_driveFeedforward.Calculate(state.speed);

    if (m_id == 1) {
      frc::SmartDashboard::PutNumber("dRate 1", m_driveMotor.GetRate());
    }
    if (m_id == 2) {
      frc::SmartDashboard::PutNumber("dRate 2", m_driveMotor.GetRate());
      // frc::SmartDashboard::PutNumber("dOut 2", driveOutput);
    }
    if (m_id == 3) {
      frc::SmartDashboard::PutNumber("dRate 3", m_driveMotor.GetRate());
    }
    if (m_id == 4) {
      frc::SmartDashboard::PutNumber("dRate 4", m_driveMotor.GetRate());
    }

      m_driveMotor.SetVoltage(units::volt_t(-driveOutput) - driveFF);
      m_turningMotor.Set(turnOutput);
}

void SwerveModule::ResetEncoders() {
  m_driveMotor.SetPosition(0);
}

units::celsius_t SwerveModule::GetDriveMotorTemp() {
  return units::celsius_t(m_driveMotor.GetMotorTemperature());
}

units::celsius_t SwerveModule::GetTurnMotorTemp() {
  return units::celsius_t(m_turningMotor.GetMotorTemperature());
}
