package org.firstinspires.ftc.teamcode.messages;

import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;

public final class TwoDeadWheelEncodersMessage {
    public long timestamp;
    public int rawParPosition;
    public int rawParVelocity;
    public int rawPerpPosition;
    public int rawPerpVelocity;

    public TwoDeadWheelEncodersMessage(PositionVelocityPair par, PositionVelocityPair perp) {
        this.timestamp = System.nanoTime();
        this.rawParPosition = par.rawPosition;
        this.rawParVelocity = par.rawVelocity;
        this.rawPerpPosition = perp.rawPosition;
        this.rawPerpVelocity = perp.rawVelocity;
    }
}
