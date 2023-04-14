#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <cmath>
#include <numbers>
#include <string>

#include "utils/cams/Limelight.h"

#define GetVal(x) nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber(x , 0.0)
#define SetVal(x , y) nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber(x , y)

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

void limeLight::SetLED(LEDMode Mode) {
  SetVal("ledMode", (int)Mode);
}

void limeLight::SetMode(CamMode Mode) {
  SetVal("camMode", (int)Mode);
}

void limeLight::SetPipeline(Pipeline Pipe) {
  SetVal("pipeline", (int)Pipe);
}

limeLight::Pipeline limeLight::GetPipeline() {
  return limeLight::Pipeline(GetVal("pipeline"));
}

limeLight::CamMode limeLight::GetMode() {
  return limeLight::CamMode(GetVal("camMode"));
}

limeLight::LEDMode limeLight::GetLED() {
  return limeLight::LEDMode(GetVal("ledMode"));
}