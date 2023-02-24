// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>

#include "Constants.h"

using namespace DriveConstants;
using namespace DriveConstants::CanIds;

DriveSubsystem::DriveSubsystem()
    : m_frontLeft{kFrontLeftDriveMotorPort,
                  kFrontLeftTurningMotorPort,
                  kFrontLeftTurningEncoderPorts,
                  kFrontLeftOffset},

      m_rearLeft{
          kRearLeftDriveMotorPort,       kRearLeftTurningMotorPort,
          kRearLeftTurningEncoderPorts,  kRearLeftOffset},

      m_frontRight{
          kFrontRightDriveMotorPort,       kFrontRightTurningMotorPort,
          kFrontRightTurningEncoderPorts,  kFrontRightOffset},

      m_rearRight{
          kRearRightDriveMotorPort,       kRearRightTurningMotorPort,
          kRearRightTurningEncoderPorts,  kRearRightOffset},

      m_odometry(kDriveKinematics, m_gyro.GetRotation2d(), {m_frontLeft.GetPosition(),
                    m_rearLeft.GetPosition(), m_frontRight.GetPosition(),
                    m_rearRight.GetPosition()}, frc::Pose2d()) {}

void DriveSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
  m_odometry.Update(m_gyro.GetRotation2d(), {m_frontLeft.GetPosition(),
                    m_rearLeft.GetPosition(), m_frontRight.GetPosition(),
                    m_rearRight.GetPosition()});
}

void DriveSubsystem::Drive(units::meters_per_second_t xSpeed,
                           units::meters_per_second_t ySpeed,
                           units::radians_per_second_t rot,
                           bool fieldRelative) {
  auto states = kDriveKinematics.ToSwerveModuleStates(
      fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                          xSpeed, ySpeed, rot, m_gyro.GetRotation2d())
                    : frc::ChassisSpeeds{xSpeed, ySpeed, rot});

  kDriveKinematics.DesaturateWheelSpeeds(&states, AutoConstants::kMaxSpeed);

  auto [fl, fr, bl, br] = states;

  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_rearLeft.SetDesiredState(bl);
  m_rearRight.SetDesiredState(br);
}

void DriveSubsystem::SetModuleStates(
    wpi::array<frc::SwerveModuleState, 4> desiredStates) {
  kDriveKinematics.DesaturateWheelSpeeds(&desiredStates,
                                         AutoConstants::kMaxSpeed);
  desiredStates[0].angle = desiredStates[0].angle * -1.0;
  desiredStates[1].angle = desiredStates[1].angle * -1.0;
  desiredStates[2].angle = desiredStates[2].angle * -1.0;
  desiredStates[3].angle = desiredStates[3].angle * -1.0;
  m_frontLeft.SetDesiredState(desiredStates[0]);
  m_frontRight.SetDesiredState(desiredStates[1]);
  m_rearLeft.SetDesiredState(desiredStates[2]);
  m_rearRight.SetDesiredState(desiredStates[3]);

  // printf("speed=%5.2f angle=%5.2f\n", desiredStates[0].speed, desiredStates[0].angle);
}

units::degree_t DriveSubsystem::GetHeading() const {
  return m_gyro.GetRotation2d().Degrees();
}

void DriveSubsystem::ZeroHeading() {
  m_gyro.Reset();
}

double DriveSubsystem::GetTurnRate() {
  return -m_gyro.GetRate();
}

frc::Pose2d DriveSubsystem::GetPose() {
  return m_odometry.GetPose();
}

void DriveSubsystem::ResetOdometry(frc::Pose2d pose) {
  m_odometry.ResetPosition(frc::Rotation2d(units::degree_t(m_gyro.GetAngle())), 
                    {m_frontLeft.GetPosition(), m_rearLeft.GetPosition(),
                    m_frontRight.GetPosition(), m_rearRight.GetPosition()},
                    pose);
}
