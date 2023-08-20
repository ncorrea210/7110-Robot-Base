[Back](/docs/utils/Util.md)

### [LimeLight.h](/src/main/include/utils/cams/Limelight.h) [LimeLight.cpp](/src/main/cpp/utils/cams/Limelight.cpp)

## Methods:

### LimeLight() = delete;
This is a staticly defined class so there is no constructor

### enum class LEDMode
enum containing the options for the LEDMode
- kPipeline
- kOff
- kOn

### enum class CamMode
Enum containing the camera modes
- kProcessed
- kUnProcessed

### enum class Pipeline
Enum containing all the pipelines that are defined
- kRetroReflective
- kApriltag

### bool HasTarget()
Returns true if the limelight sees a target, false otherwise. 

### double GetX()
Get how many degrees off the target is from the center in the X direction. 

### double GetY()
Get how many degrees off the target is from the center in the Y direction.

### double GetA()
Get the total area of the best target, can be used to determine distance(With little accuracy).

### void SetLED(LEDMode)
Sets the mode of the LED from the list of options in the LEDMode enum. 

### void SetMode(CamMode)
Sets the mode of the camera from the list of the options in the CamMode enum. 

### void SetPipeline(Pipeline)
Sets the pipeline of the limelight from the list of options in the Pipeline enum. 

### Pipeline GetPipeline() 
Returns the current Pipeline of the LimeLight.

### CamMode GetCamMode()
Returns the current CamMode of the LimeLight.

### LEDMode GetLED()
Returns the current state of the LED on the LimeLight. 

### std::pair<std::optional<frc::Pose2d>, std::optional<units::second_t>> GetPose()
Gets the estimated pose of the robot from the limelight's point of view. Will return nullopt for both optionals if there is no target. 
