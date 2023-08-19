#include "utils/cams/Limelight.h"

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

#include <cmath>
#include <numbers>
#include <string>
#include <vector>

#include <frc/DriverStation.h>

#define GETVAL(x) nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber(x , 0.0)
#define GET_ARRAY_VAL(x) nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumberArray(x, std::span<const double>(6))
#define SETVAL(x , y) nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber(x , y)

using namespace hb;

bool LimeLight::HasTarget() {
  return (bool)GETVAL("tv");
}

double LimeLight::GetX() {
  return GETVAL("tx");
};

double LimeLight::GetY() {
  return GETVAL("ty");
}

double LimeLight::GetA() {
  return GETVAL("tx");
}

void LimeLight::SetLED(LEDMode Mode) {
  SETVAL("ledMode", (int)Mode);
}

void LimeLight::SetMode(CamMode Mode) {
  SETVAL("camMode", (int)Mode);
}

void LimeLight::SetPipeline(Pipeline Pipe) {
  SETVAL("pipeline", (int)Pipe);
}

LimeLight::Pipeline LimeLight::GetPipeline() {
  return LimeLight::Pipeline(GETVAL("pipeline"));
}

LimeLight::CamMode LimeLight::GetMode() {
  return LimeLight::CamMode(GETVAL("camMode"));
}

LimeLight::LEDMode LimeLight::GetLED() {
  return LimeLight::LEDMode(GETVAL("ledMode"));
}

std::pair<std::optional<frc::Pose2d>, std::optional<units::second_t>> LimeLight::GetPose() {
  static std::vector<double> results;
  if (frc::DriverStation::GetAlliance() != frc::DriverStation::Alliance::kBlue) {
    results = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumberArray("botpose_wpiblue", std::span<const double>());
  } else {
    results = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumberArray("botpose_wpired", std::span<const double>());
  }
  if (!HasTarget()) {
    return std::make_pair(std::nullopt, std::nullopt);
  }
  frc::Translation2d translation{units::meter_t(results[0]), units::meter_t(results[1])};
  frc::Rotation2d rotation{units::degree_t(results[5])};
  frc::Pose2d pose{translation, rotation};
  units::second_t timestamp{results[6]};

  return std::make_pair(pose, timestamp);
}

// std::array<double, 6> limeLight::GetBotpose() {
//   static std::array<double, 6> arr;
//   for (int i = 0; i < 6; i++) {
//     arr[i] = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetValue("botpose").GetDoubleArray()[i];
//   }
//   return arr;
// }

// frc::Translation2d limeLight::GetBotPose2D() {
//   return frc::Translation2d(units::meter_t(GetBotpose()[0]), -units::meter_t(GetBotpose()[1]));
// }