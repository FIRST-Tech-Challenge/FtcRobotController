package org.firstinspires.ftc.teamcode.messages;

import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.Time;

public final class DriveCommandMessage {
    public long timestamp;
    public double forwardVelocity;
    public double forwardAcceleration;
    public double lateralVelocity;
    public double lateralAcceleration;
    public double angularVelocity;
    public double angularAcceleration;

    public DriveCommandMessage(PoseVelocity2dDual<Time> poseVelocity) {
        this.timestamp = System.nanoTime();
        this.forwardVelocity = poseVelocity.linearVel.x.get(0);
        this.forwardAcceleration = poseVelocity.linearVel.x.get(1);
        this.lateralVelocity = poseVelocity.linearVel.y.get(0);
        this.lateralAcceleration = poseVelocity.linearVel.y.get(1);
        this.angularVelocity = poseVelocity.angVel.get(0);
        this.angularAcceleration = poseVelocity.angVel.get(1);
    }
}
