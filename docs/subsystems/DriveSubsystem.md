[Back](/docs/subsystems/Subsystems.md)

### [DriveSubsystem.h](/src/main/include/subsystems/DriveSubsystem.h) [DriveSubsystem.cpp](/src/main/cpp/subsystems/DriveSubsystem.cpp)

## Methods:

### DriveSubsystem()
The contructor does not take in any arguements. All initializing values come from Constants.h in the DriveContants namespace

### void Periodic()
The periodic function contains anything that needs to be called constantly. In the case of the drive subsystem, it updates the pose estimation on the robot

### void Drive(units::meters_per_second_t xSpeed, units::meters_per_second_t ySpeed, units::radians_per_second_t rot, bool fieldRelative)
Drives the robot with either field relative or without it. x is the forward direction and y is the sideways direction. 

### void ResetEncoders()
Resets all the swerve module encoders. (Sets distance travled to zero and re-zeros the angle of the motors with the CANCoder)

### void SetModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredstates)
Sets the states of each module individually based on what's entered in the array

### void ZeroHeading()
Zeros the gyro to 0 for wherever it's facing

### double GetTurnRate()
Returns the rotation speed of the robot in degree per second

### frc::Pose2d GetPose()
Returns the pose of the robot. Pose2d gives the x location, y location, and rotation of the robot. 

### void SetPose(frc::Pose2d)
Sets the pose of the robot. This is a good way to update the robot's location if you can for sure know where it is. 

### void ResetOdometry(frc::Pose2d)
This resets the odometry of the robot. This really isn't needed anymore and should be delted soon. 

### frc::HolonomicDriveController GetController()
This returns the drive controller that allows the control of the robot in autonomous. This still has to be applied with SetModuleStates, this just allows for a globabl controller. 

### void ToggleVision()
Toggles the vision on and off. When the vision is off, the vision estimations won't be added to the pose estimation

### void ResetGyro()
Resets the gyro's heading to whever it's facing.

### void InitSendable(wpi::SendableBuilder)
Initializes the smartdashboard sendable with any info that needs to be updated. 

## Devices
* Swerve Modules x 4
* Pigeon Gyro