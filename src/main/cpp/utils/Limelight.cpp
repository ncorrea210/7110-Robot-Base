#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <cmath>
#include <numbers>
#include <string>

#include "utils/Limelight.h"

#define GetVal(x) NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber(x , 0.0)
#define SetVal(x , y) NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber(x , y)
#define ToRadians(x) ((x/180) * std::numbers::pi)
#define ToDegrees(x) ((x/std::numbers::pi) * 180)

using namespace nt;
using namespace hb;


bool limeLight::HasTarget() {
  return (bool)GetVal("tv");
}

double limeLight::GetX() {
  return GetVal("tx");
};

double limeLight::GetY() {
  return GetVal("ty");
}

LimeLightVec limeLight::GetVec() {
  LimeLightVec Vec;
  Vec.distance = std::sqrt((GetX() * GetX()) + (GetY() * GetY()));
  Vec.angle = ToDegrees(atan(GetY() / GetX()));
  return Vec;
}

void limeLight::SetLED(LEDMode Mode) {
  SetVal("ledMode", (int)Mode);
}

void limeLight::SetMode(CamMode Mode) {
  SetVal("camMode", (int)Mode);
}

void limeLight::SetPipeline(Pipeline Pipe) {
  SetVal("pipeline", (int)Pipe);
}