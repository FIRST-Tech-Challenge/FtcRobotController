package org.firstinspires.ftc.team6220_CENTERSTAGE;

import com.acmerobotics.roadrunner.Pose2d;

/*
This is part of roadrunner.
 */
public final class PoseMessage {
    public long timestamp;
    public double x;
    public double y;
    public double heading;

    public PoseMessage(Pose2d pose) {
        this.timestamp = System.nanoTime();
        this.x = pose.position.x;
        this.y = pose.position.y;
        this.heading = pose.heading.log();
    }
}

