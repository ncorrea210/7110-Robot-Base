#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <cmath>
#include <numbers>
#include <string>
#include <vector>

#include <frc/DriverStation.h>

#include "utils/cams/Limelight.h"

#define GETVAL(x) nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber(x , 0.0)
#define GET_ARRAY_VAL(x) nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumberArray(x, std::span<const double>(6))
#define SETVAL(x , y) nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber(x , y)

using namespace hb;


bool limeLight::HasTarget() {
  return (bool)GETVAL("tv");
}

double limeLight::GetX() {
  return GETVAL("tx");
};

double limeLight::GetY() {
  return GETVAL("ty");
}

double limeLight::GetA() {
  return GETVAL("tx");
}

void limeLight::SetLED(LEDMode Mode) {
  SETVAL("ledMode", (int)Mode);
}

void limeLight::SetMode(CamMode Mode) {
  SETVAL("camMode", (int)Mode);
}

void limeLight::SetPipeline(Pipeline Pipe) {
  SETVAL("pipeline", (int)Pipe);
}

limeLight::Pipeline limeLight::GetPipeline() {
  return limeLight::Pipeline(GETVAL("pipeline"));
}

limeLight::CamMode limeLight::GetMode() {
  return limeLight::CamMode(GETVAL("camMode"));
}

limeLight::LEDMode limeLight::GetLED() {
  return limeLight::LEDMode(GETVAL("ledMode"));
}

std::pair<std::optional<frc::Pose2d>, std::optional<units::second_t>> limeLight::GetPose() {
  static std::vector<double> results;
  if (frc::DriverStation::GetAlliance() != frc::DriverStation::Alliance::kBlue) {
    results = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumberArray("botpose_wpiblue", std::span<const double>());
  } else {
    results = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumberArray("botpose_wpired", std::span<const double>());
  }
  if (results[6] == 0) {
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