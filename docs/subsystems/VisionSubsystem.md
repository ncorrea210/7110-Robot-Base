[Back](/docs/subsystems/Subsystems.md)

## [VisionSubsystem.h](/src/main/include/subsystems/VisionSubsystem.h) [VisionSubsystem.cpp](/src/main/cpp/subsystems/VisionSubsystem.cpp)

***This subsystem cannot be initialized, it has a static "GetInstance" funtion that allows you to get a reference to the one instance of the class***

## Methods:

### VisionSubsystem()
All initializing values come from Constants.h in VisionConstants. 

### void Periodic()
This function does nothing in this subsystem.

### VisionSubsystem& GetInstance()
Returns an instance of the subsystem. 

### photonlib::PhotonCamera& GetLeftCam()
Returns a reference to the left camera so you can do whatever you need to do with it. 

### photonlib::PhotonCamera& GetRightCam()
Returns a reference to the right camera so you can do whatver you need to do with it. 

### photonlib::PhotonPipelineResult GetLeftFrame()
Returns the latest frame from the left Camera. 

### photonlib::PhotonPipelineResule GetRightFrame()
Returns the latest frame from the right Camera.

### std::pair<std::optional<units::second_t>, std::optional<frc::Pose2d>> GetPose()
Returns the pose estimated by the cameras. Will return nullopt for both if there is no pose estimated.

### void initSendable(wpi::SendableBuilder)
Initializes the smartdashboard sendable with any info that needs to be updated. 
