#include "commands/AutoRoutines.h"
#include "Constants.h"

using namespace pathplanner;

Auto::Auto(){}

frc::Trajectory Auto::GetTrajectory1() {
  PathPlannerTrajectory path = PathPlanner::loadPath("TestPath", AutoConstants::kMaxSpeed, AutoConstants::kMaxAcceleration);
  return path.asWPILibTrajectory();
  }
  