// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <optional>
#include <utility>

#include <frc2/command/SubsystemBase.h>

#include <frc/geometry/Pose2d.h>
#include <frc/apriltag/AprilTagFields.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/smartdashboard/Field2d.h>

#include <wpi/sendable/SendableBuilder.h>

#include <units/time.h>

#include <photonlib/PhotonCamera.h>
#include <photonlib/PhotonPoseEstimator.h>

class VisionSubsystem : public frc2::SubsystemBase {
 public:
  VisionSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  static VisionSubsystem& GetInstance();

  // photonlib::PhotonCamera& GetRightCam();

  // static photonlib::PhotonCamera& GetLeftCam();

  std::pair<std::optional<units::second_t>, std::optional<frc::Pose2d>> GetPose();

  // double GetLeftX();

  // double GetLeftY();

  void InitSendable(wpi::SendableBuilder& builder) override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  frc::AprilTagFieldLayout m_layout = frc::LoadAprilTagLayoutField(frc::AprilTagField::k2023ChargedUp);

  // photonlib::PhotonCamera m_rightCam;
  // photonlib::PhotonCamera m_leftCam;

  photonlib::PhotonPoseEstimator m_rightEst;
  photonlib::PhotonPoseEstimator m_leftEst;

  frc::Field2d m_field;

  // photonlib::PhotonPipelineResult m_leftResult;
};
