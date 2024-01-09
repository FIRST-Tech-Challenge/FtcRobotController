package org.firstinspires.ftc.teamcode.messages;

import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;

public final class MecanumEncodersMessage {
    public long timestamp;
    public PositionVelocityPair leftFront;
    public PositionVelocityPair leftBack;
    public PositionVelocityPair rightBack;
    public PositionVelocityPair rightFront;

    public MecanumEncodersMessage(PositionVelocityPair leftFront, PositionVelocityPair leftBack, PositionVelocityPair rightBack, PositionVelocityPair rightFront) {
        this.timestamp = System.nanoTime();
        this.leftFront = leftFront;
        this.leftBack = leftBack;
        this.rightBack = rightBack;
        this.rightFront = rightFront;
    }
}
