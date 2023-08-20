[Back](/docs/subsystems/Subsystems.md)

### [ClawSubsystem.h](/src/main/include/subsystems/ClawSubsystem.h) [ClawSubsystem.cpp](/src/main/cpp/subsystems/ClawSubsystem.cpp)

## Methods

### ClawSubsystem(frc::PowerDistribution* pdp) 
- pdp: A pointer to the power distribution panel 
Most initializing values come directly from the Constants.h file. 

### void Periodic()
Periodically runs at 50hz updating anything that needs to be updated for the subsystem. This does not do anything right now. 

### void Run(double val) 
Runs the claw at the value set by val. Must be bounded from [-1,1]

### void InitSendable(wpi::SendableBuilder)
Initializes the smartdashboard sendable with any info that needs to be updated. 

## Devices:
- Bag motor