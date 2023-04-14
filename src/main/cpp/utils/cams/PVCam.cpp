// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "utils/cams/PVCam.h"

using namespace hb;

PVCam::PVCam(std::string name) : photonlib::PhotonCamera(name) {}

void PVCam::UpdateResults() {
  m_result = GetLatestResult();
}

photonlib::PhotonPipelineResult PVCam::GetResult() {
  return m_result;
}