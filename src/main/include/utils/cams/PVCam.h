// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <photonlib/PhotonCamera.h>
#include <string>

namespace hb {
  class PVCam : public photonlib::PhotonCamera {
    public:

      explicit PVCam(std::string);

      void UpdateResults();

      photonlib::PhotonPipelineResult GetResult();

    private:
      photonlib::PhotonPipelineResult m_result;
  };
}
