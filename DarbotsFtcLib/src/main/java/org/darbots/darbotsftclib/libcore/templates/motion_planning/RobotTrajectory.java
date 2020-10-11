package org.darbots.darbotsftclib.libcore.templates.motion_planning;

import org.darbots.darbotsftclib.libcore.motion_planning.trajectories.TrajectoryMotionState;

public interface RobotTrajectory {
    RobotMotionProfilingIterator<TrajectoryMotionState,?> getIterator();
    TrajectoryMotionState getEndState();
}
