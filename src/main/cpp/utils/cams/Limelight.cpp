#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <cmath>
#include <numbers>
#include <string>
#include <vector>

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