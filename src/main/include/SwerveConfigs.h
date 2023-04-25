#pragma once

#include "utils/swerve/Config.h"

class SwerveConfigs {
  public:

    static SwerveConfigs& GetInstance();

    SwerveConfig GetFrontLeft();

    SwerveConfig GetFrontRight();

    SwerveConfig GetRearLeft();

    SwerveConfig GetRearRight();

  private: 
    SwerveConfigs();

    SwerveConfig m_FrontLeft;
    SwerveConfig m_FrontRight;
    SwerveConfig m_RearLeft;
    SwerveConfig m_RearRight;
};