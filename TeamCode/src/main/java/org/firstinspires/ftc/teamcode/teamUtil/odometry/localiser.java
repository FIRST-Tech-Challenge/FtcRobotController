package org.firstinspires.ftc.teamcode.teamUtil.odometry;

import org.firstinspires.ftc.teamcode.teamUtil.pose2D;

public interface localiser {

    pose2D poseEstimate();
    pose2D poseVelocity();

    void update();
}
