#pragma once

#include <array>
#include <optional>
#include <utility>

#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Pose2d.h>

#include <units/time.h>

namespace hb {
  
  class LimeLight {
    
    public:

      explicit LimeLight() = delete;

      enum class LEDMode {
        kPipeline = 0,
        kOff = 1,
        kBlink = 2,
        kOn = 3
      };

      enum class CamMode {
        kProcessed = 0,
        kUnprocessed = 1
      };
      
      /**
       * enum class for easier naming for pipelines
      */
      enum class Pipeline {
        kRetroReflective = 0,
        kAprilTag = 1
      };

      /**
       * @returns true if a target is spotted
      */
      static bool HasTarget();

      /**
       * @returns The X offset to target
      */
      static double GetX();

      /**
       * @returns The Y offset to target
      */
      static double GetY();

      /**
       * @returns The area the target covers, meant for apriltags
      */
      static double GetA();

      /**
       * @param LEDMode enum class sets the LED on, off, or to the pipeline
      */
      static void SetLED(LEDMode); 

      /**
       * @param CAMMode enum class 
       * 
       * @brief sets the Camera from processed to unprocessed
      */
      static void SetMode(CamMode);

      /**
       * @param Pipeline enum class
       * sets the camera to a specific pipeline ie Apriltag or retro reflective tape
      */
      static void SetPipeline(Pipeline);

      static Pipeline GetPipeline();

      static CamMode GetMode();

      static LEDMode GetLED();

      static std::pair<std::optional<frc::Pose2d>, std::optional<units::second_t>> GetPose();

      // static std::array<double, 6> GetBotpose(); 

      // static frc::Translation2d GetBotPose2D();

  };
}

