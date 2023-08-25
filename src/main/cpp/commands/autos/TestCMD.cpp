// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// #include "commands/autos/TestCMD.h"

// #include <pathplanner/lib/auto/SwerveAutoBuilder.h>
// #include <pathplanner/lib/PathPlanner.h>

// using namespace pathplanner;

// // NOTE:  Consider using this command inline, rather than writing a subclass.
// // For more information, see:
// // https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
// TestCMD::TestCMD(DriveSubsystem* drive) {
//   // Add your commands here, e.g.

// std::unordered_map<std::string, std::shared_ptr<frc2::Command>> eventMap;

// // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this could be in RobotContainer along with your subsystems

// SwerveAutoBuilder autoBuilder(
//     [drive]() { return drive->GetPose(); }, // Function to supply current robot pose
//     [drive](auto initPose) { drive->SetPose(initPose); }, // Function used to reset odometry at the beginning of auto
//     PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
//     PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
//     [drive](auto speeds) { drive->DriveFieldRelative(speeds); }, // Output function that accepts field relative ChassisSpeeds
//     eventMap, // Our event map
//     { drive }, // Drive requirements, usually just a single drive subsystem
//     true // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
// );

// auto path = PathPlanner::loadPath("BackForth", PathConstraints(units::meters_per_second_t(1.5), units::meters_per_second_squared_t(4)));
// frc2::CommandPtr cmd = autoBuilder.followPath(path);

// AddCommands(
//   *cmd.get()
// );

//   // AddCommands(FooCommand{}, BarCommand{});
// }
