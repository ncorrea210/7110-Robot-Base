// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/ADXRS450_Gyro.h>
#include <frc/Encoder.h>
#include <frc/drive/MecanumDrive.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/interfaces/Gyro.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc2/command/SubsystemBase.h>
#include <wpi/array.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/DigitalInput.h>
#include <units/angle.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/HolonomicDriveController.h>
#include <array>


#include "Constants.h"
#include "utils/SwerveModule.h"
#include "utils/PigeonGyro.h"

typedef struct {
  double speed;
  double angle;
} VecDrive;

class DriveSubsystem : public frc2::SubsystemBase {
 public:
  DriveSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  // Subsystem methods go here.

  /**
   * Drives the robot at given x, y and theta speeds. Speeds range from [-1, 1]
   * and the linear speeds have no effect on the angular speed.
   *
   * @param xSpeed        Speed of the robot in the x direction
   *                      (forward/backwards).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to
   *                      the field.
   */
  void Drive(units::meters_per_second_t xSpeed,
             units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
             bool fieldRelative);

  void Drive(VecDrive Drive, units::radians_per_second_t rot, bool fieldRelative);

  void Drive(VecDrive Drive, units::radian_t heading);

  void Drive(units::meters_per_second_t xSpeed, units::meters_per_second_t ySpeed, units::radian_t heading);

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  void ResetEncoders();

  /**
   * Sets the drive MotorControllers to a power from -1 to 1.
   */
  void SetModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates);


  /**
   * Zeroes the heading of the robot.
   */
  void ZeroHeading();

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  double GetTurnRate();

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  frc::Pose2d GetPose();

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  void ResetOdometry(frc::Pose2d pose);

  frc::HolonomicDriveController GetController() {
    return frc::HolonomicDriveController{
      frc2::PIDController{AutoConstants::kPXController, 0, 0},
      frc2::PIDController{AutoConstants::kPYController, 0, 0},
      frc::ProfiledPIDController<units::radian>{
        1, 0, 0, AutoConstants::kThetaControllerConstraints
      }};
  }

  void SetSpeed(const double& speed) {
    m_speed = speed;
  }

  units::meters_per_second_t GetSpeed() {
    return m_speed > DriveConstants::kMaxSpeed.value() ? 
    DriveConstants::kMaxSpeed : units::meters_per_second_t(m_speed);
  }

  units::meter_t kTrackWidth =
      0.31369_m;  // Distance between centers of right and left wheels on robot
  units::meter_t kWheelBase =
      0.31369_m;  // Distance between centers of front and back wheels on robot

  frc::SwerveDriveKinematics<4> kDriveKinematics{
      frc::Translation2d(kWheelBase / 2, -kTrackWidth / 2),
      frc::Translation2d(kWheelBase / 2, kTrackWidth / 2),
      frc::Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
      frc::Translation2d(-kWheelBase / 2, kTrackWidth / 2)};

    void ResetGyro() {
        gyro.Reset();
    }

  hb::pigeonGyro gyro{DriveConstants::CanIds::kPidgeonID};

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  SwerveModule m_frontLeft;
  SwerveModule m_rearLeft;
  SwerveModule m_frontRight;
  SwerveModule m_rearRight;

  double m_speed;

  frc::ProfiledPIDController<units::radians> m_turnController{7.5, 0, 0,
   {DriveConstants::kMaxAngularSpeed, DriveConstants::kMaxAngularAcceleration}};

  // Odometry class for tracking robot pose
  // 4 defines the number of modules
  frc::SwerveDriveOdometry<4> m_odometry;
};
