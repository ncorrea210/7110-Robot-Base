// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SwerveModule.h"

#include <frc/geometry/Rotation2d.h>
#include <numbers>

#include "Constants.h"

SwerveModule::SwerveModule(int driveMotorChannel, int turningMotorChannel,
                           const int turningEncoderPorts,
                           const double offset)
    : m_driveMotor(driveMotorChannel, rev::CANSparkMax::MotorType::kBrushless),
      m_turningMotor(turningMotorChannel, rev::CANSparkMax::MotorType::kBrushless),
      m_turningEncoder(turningEncoderPorts, offset),
      m_id(turningEncoderPorts) {
  // Set the distance per pulse for the drive encoder. We can simply use the
  // distance traveled for one rotation of the wheel divided by the encoder
  // resolution.
  m_driveMotor.SetRPM2MPS(
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
    return {m_driveMotor.GetDistance(), frc::Rotation2d(units::radian_t(m_turningEncoder.Get()))};
}

void SwerveModule::SetDesiredState(
    const frc::SwerveModuleState& referenceState) {
  // Optimize the reference state to avoid spinning further than 90 degrees
  const auto state = frc::SwerveModuleState::Optimize(
      referenceState, units::radian_t(m_turningEncoder.Get()));

  // Calculate the drive output from the drive PID controller.
  const auto driveOutput = m_drivePIDController.Calculate(
      m_driveMotor.GetRate(), state.speed.value());

  // Calculate the turning motor output from the turning PID controller.
  auto turnOutput = m_turningPIDController.Calculate(
      units::radian_t(m_turningEncoder.Get()), state.angle.Radians());

  if (m_id == 0) printf("Id: %d Get: %5.2f state: %5.2f | ", m_id, m_turningEncoder.Get(), state.angle.Radians().value());
  if (m_id == 2) printf("Id: %d Get: %5.2f state: %5.2f\n", m_id, m_turningEncoder.Get(), state.angle.Radians().value());

  units::volt_t driveFF = m_driveFeedforward.Calculate(state.speed);

  // Set the motor outputs.
  // if(fabs(state.speed.value()) < 0.001) {
  //   m_driveMotor.Set(0);
  //   m_turningMotor.Set(0);
  // }
  // else {
  //   m_driveMotor.SetVoltage(units::volt_t(driveOutput) + driveFF);
  //   m_turningMotor.Set(turnOutput);
  // }
    m_driveMotor.SetVoltage(units::volt_t(driveOutput) + driveFF);
    m_turningMotor.Set(turnOutput);
}
