package org.firstinspires.ftc.teamcode.messages;

import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;

public final class MecanumEncodersMessage {
    public long timestamp;
    public int rawLeftFrontPosition;
    public int rawLeftFrontVelocity;
    public int rawLeftBackPosition;
    public int rawLeftBackVelocity;
    public int rawRightBackPosition;
    public int rawRightBackVelocity;
    public int rawRightFrontPosition;
    public int rawRightFrontVelocity;

    public MecanumEncodersMessage(PositionVelocityPair leftFront, PositionVelocityPair leftBack, PositionVelocityPair rightBack, PositionVelocityPair rightFront) {
        this.timestamp = System.nanoTime();
        this.rawLeftFrontPosition = leftFront.rawPosition;
        this.rawLeftFrontVelocity = leftFront.rawVelocity;
        this.rawLeftBackPosition = leftBack.rawPosition;
        this.rawLeftBackVelocity = leftBack.rawVelocity;
        this.rawRightBackPosition = rightBack.rawPosition;
        this.rawRightBackVelocity = rightBack.rawVelocity;
        this.rawRightFrontPosition = rightFront.rawPosition;
        this.rawRightFrontVelocity = rightFront.rawVelocity;
    }
}
