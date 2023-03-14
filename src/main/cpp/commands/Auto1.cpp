#include "commands/Auto1.h"
#include "Constants.h"


using namespace pathplanner;

frc::Trajectory path1() {
    PathPlannerTrajectory p1 = PathPlanner::loadPath("TPath", 
    PathConstraints(AutoConstants::kMaxSpeed, AutoConstants::kMaxAcceleration));
    return p1.asWPILibTrajectory();
}