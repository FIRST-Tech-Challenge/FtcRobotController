package org.firstinspires.ftc.teamcode.teamUtil.trajectoryAssembly;

import org.firstinspires.ftc.teamcode.teamUtil.*;

public class TrajectorySegment {
    SegmentType segmentType;
    Pose2D startPose2D;
    Pose2D endPose2D;
    double distance;
    double turnPower;
    double velocity;

    TrajectorySegment(Pose2D startPose2D, Pose2D endPose2D){
        this.segmentType = SegmentType.LINE;
        this.startPose2D = startPose2D;
        this.endPose2D = endPose2D;
        this.distance = startPose2D.getDifference(endPose2D);
        this.turnPower = 0.9;
        this.velocity = RobotConstants.maxSwerveVelocity;
    }
}

enum SegmentType {
    LINE, CURVE
}