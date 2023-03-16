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


        inline std::string sanitizeName(const std::string &name)
    {
        if (name == "")
        {
            return "limelight";
        }
        return name;
    }


    inline std::shared_ptr<nt::NetworkTable> getLimelightNTTable(const std::string &tableName)
    {
        return nt::NetworkTableInstance::GetDefault().GetTable(sanitizeName(tableName));
    }

    inline nt::NetworkTableEntry getLimelightNTTableEntry(const std::string &tableName, const std::string &entryName)
    {
        return getLimelightNTTable(tableName)->GetEntry(entryName);
    }

    
    inline void setLimelightNTDouble(const std::string &tableName, const std::string entryName, double val)
    {
        getLimelightNTTableEntry(tableName, entryName).SetDouble(val);
    }


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