#pragma once

namespace hb {
  
  class limeLight {
    
    public:

      explicit limeLight() = delete;

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
       * @param LEDMode enum class sets the LED on, off, or to the pipeline
      */
      static void SetLED(LEDMode); 

      /**
       * @param CAMMode enum class 
       * sets the Camera from processed to unprocessed
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

  };
}

