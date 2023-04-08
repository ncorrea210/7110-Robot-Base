#pragma once

#include <frc2/command/SubsystemBase.h>
#include <string>
#include <unordered_map>

namespace hb {
  class Subsystem : public frc2::SubsystemBase {
    public:

      /**
       * @brief Includes a map of labels and values for any item to be put on shuffleboard
       * 
       * @returns unodered_map of the label and value for any item
      */
      virtual std::unordered_map<std::string, double> GetTelemetry() = 0;

      /**
       * @brief function that should be used to actually place values into map
      */
      virtual void UpdateTelemetry() = 0;

    protected:
      explicit Subsystem();
  };
}