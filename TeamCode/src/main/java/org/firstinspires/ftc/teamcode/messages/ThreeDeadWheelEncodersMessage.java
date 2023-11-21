package org.firstinspires.ftc.teamcode.messages;

import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;

public final class ThreeDeadWheelEncodersMessage {
    public long timestamp;
    public int rawPar0Position;
    public int rawPar0Velocity;
    public int rawPar1Position;
    public int rawPar1Velocity;
    public int rawPerpPosition;
    public int rawPerpVelocity;

    public ThreeDeadWheelEncodersMessage(PositionVelocityPair par0, PositionVelocityPair par1, PositionVelocityPair perp) {
        this.timestamp = System.nanoTime();
        this.rawPar0Position = par0.rawPosition;
        this.rawPar0Velocity = par0.rawVelocity;
        this.rawPar1Position = par1.rawPosition;
        this.rawPar1Velocity = par1.rawVelocity;
        this.rawPerpPosition = perp.rawPosition;
        this.rawPerpVelocity = perp.rawVelocity;
    }
}