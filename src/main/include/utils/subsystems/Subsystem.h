#pragma once

#include <frc2/command/SubsystemBase.h>
#include <string>
#include <unordered_map>
#include <functional>

namespace hb {

  typedef struct SubsystemData{
    std::string name;
    std::unordered_map<std::string, std::function<double()>> telemetry;
  } SubsystemData;

  class Subsystem : public frc2::SubsystemBase {
    public:

      /**
       * @brief Includes a map of labels and values for any item to be put on shuffleboard
       * 
       * @returns unodered_map of the label and value for any item
      */
      virtual std::unordered_map<std::string, std::function<double()>> GetTelemetry() = 0;

      /**
       * @brief function that should be used to actually place values into map
      */
      virtual void SetTelemetry() = 0;

      virtual SubsystemData GetData() = 0;

    protected:
      explicit Subsystem();
  };
}