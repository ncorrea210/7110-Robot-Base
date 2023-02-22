#pragma once

#include <pathplanner/lib/PathPlanner.h>
#include <frc/trajectory/Trajectory.h>

class Auto {
  public:
    Auto();

    /**
     * @brief Gets the trajectory of Pathplanner Path as frc::Trajectory
     * 
     * @return frc::Trajectory (the path)
     */
    frc::Trajectory GetTrajectory1();

};