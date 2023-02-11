package org.firstinspires.ftc.teamcode.teamUtil.odometry;

import org.firstinspires.ftc.teamcode.teamUtil.pose2D;

import java.util.List;

/**
 * unfinished, just gonna import roadrunner nad lock a bunch of stuff up so we cant access it and only grab the odometry tracking
 */

public abstract class threeTrackingWheelLocaliser implements localiser{
    public threeTrackingWheelLocaliser(List<pose2D> wheelPoses) {
        this.wheelPoses = wheelPoses;
    }
    private final List<pose2D> wheelPoses;
    private final pose2D _poseEstimate = new pose2D();

    @Override
    public pose2D poseEstimate() {
        return new pose2D();
    }
}
