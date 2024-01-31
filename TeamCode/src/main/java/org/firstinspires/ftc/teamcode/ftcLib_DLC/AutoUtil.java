package org.firstinspires.ftc.teamcode.ftcLib_DLC;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class AutoUtil {
    /*static final int CONE_CYCLES = 1;
    static final Pose2d startPose = new Pose2d(36, -55, Math.toRadians(270));
    static final Pose2d uptoPose = new Pose2d(36, -10.5, Math.toRadians(0));
    static final Pose2d park = new Pose2d(-60, -55.0, Math.toRadians(270));
    // rr

    // timing in milliseconds
    static final int INTAKE_EXTEND_TIME = 3200;
    static final int CLAW_CLEARS_DRIVETRAIN = 3000;
    static final int DROP_TIME = 3000;*/

    public double mirrorAngle(double angle) {
        return 180 - angle;
    }

    public Pose2d mirror(Pose2d pose2d) {
        Pose2d flipped = new Pose2d(pose2d.getX(), -pose2d.getY(), mirrorAngle(pose2d.getHeading()));
        return flipped;
    }
}
