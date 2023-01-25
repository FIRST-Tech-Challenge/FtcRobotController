package org.firstinspires.ftc.teamcode.teamUtil.trajectoryAssembly;

import org.firstinspires.ftc.teamcode.teamUtil.*;
import org.firstinspires.ftc.teamcode.teamUtil.trajectoryAssembly.trajectoryMarkers.trajectoryMarker;

import java.util.ArrayList;
import java.util.List;

public class trajectorySegment {
    segmentType segmentType;
    pose2D startPose2D;
    pose2D endPose2D;
    double distance;
    double turnPower;
    double velocity;

    trajectorySegment(pose2D startPose2D, pose2D endPose2D){
        this.segmentType = org.firstinspires.ftc.teamcode.teamUtil.trajectoryAssembly.segmentType.LINE;
        this.startPose2D = startPose2D;
        this.endPose2D = endPose2D;
        this.distance = startPose2D.getDifference(endPose2D);
        this.turnPower = 0;
        this.velocity = robotConstants.maxSwerveVelocity;
    }

    trajectorySegment(pose2D startPose2D, pose2D endPose2D, double turnPower){
        this.segmentType = org.firstinspires.ftc.teamcode.teamUtil.trajectoryAssembly.segmentType.LINE;
        this.startPose2D = startPose2D;
        this.endPose2D = endPose2D;
        this.distance = startPose2D.getDifference(endPose2D);
        this.turnPower = turnPower;
        this.velocity = robotConstants.maxSwerveVelocity;
    }
}

enum segmentType {
    LINE, CURVE
}