package org.firstinspires.ftc.teamcode.commandBased.classes;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class Pose2dSpline {

    private final Pose2d pose;
    private final double radians;

    public Pose2dSpline(Pose2d pose, double radians) {
        this.pose = pose;
        this.radians = radians;
    }

    public Pose2d getPose() {
        return pose;
    }

    public double getRadians() {
        return radians;
    }
}
