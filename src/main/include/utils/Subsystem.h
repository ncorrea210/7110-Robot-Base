#pragma once

#include <frc2/command/SubsystemBase.h>
#include <string>
#include <unordered_map>

namespace hb {
  class Subsystem : public frc2::SubsystemBase {
    public:

      virtual std::unordered_map<std::string, double> GetTelemetry() = 0;

      virtual void UpdateTelemetry();

    protected:
      explicit Subsystem();
  };
}