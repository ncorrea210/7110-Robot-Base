// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/VisionSubsystem.h"
#include "Constants.h"

#include <units/length.h>
#include <units/angle.h>



VisionSubsystem::VisionSubsystem() : 
m_rightCam("RightCamera"), 
m_rightEst(m_layout, photonlib::PoseStrategy::MULTI_TAG_PNP, std::move(m_rightCam), VisionConstants::m_rightTransform)
{
    m_rightEst.SetMultiTagFallbackStrategy(photonlib::PoseStrategy::LOWEST_AMBIGUITY);
    
}

// This method will be called once per scheduler run
void VisionSubsystem::Periodic() {}

VisionSubsystem& VisionSubsystem::GetInstance() {
    static VisionSubsystem inst;
    return inst;
}

photonlib::PhotonCamera& VisionSubsystem::GetRightCam() {
    return m_rightCam;
}

std::pair<std::optional<units::second_t>, std::optional<frc::Pose2d>> VisionSubsystem::GetPose() {
    std::optional<photonlib::EstimatedRobotPose> est = m_rightEst.Update();

    if (est.has_value() == false) {
        return std::make_pair(std::nullopt, std::nullopt);
    } else {
        units::meter_t x = est.value().estimatedPose.X();
        units::meter_t y = est.value().estimatedPose.Y();
        units::radian_t rot = est.value().estimatedPose.Rotation().Z(); 
        frc::Pose2d pose{x, y, rot};
        units::second_t timestamp = est.value().timestamp;
        return std::make_pair(timestamp, pose);
    }

}


