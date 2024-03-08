package org.firstinspires.ftc.teamcode.RoadRunner.messages;

import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public final class TwoDeadWheelInputsMessage {
    public long timestamp;
    public PositionVelocityPair par;
    public PositionVelocityPair perp;
    public double yaw;
    public double pitch;
    public double roll;
    public double xRotationRate;
    public double yRotationRate;
    public double zRotationRate;

    public TwoDeadWheelInputsMessage(PositionVelocityPair par, PositionVelocityPair perp, YawPitchRollAngles angles, AngularVelocity angularVelocity) {
        this.timestamp = System.nanoTime();
        this.par = par;
        this.perp = perp;
        {
            this.yaw = angles.getYaw(AngleUnit.RADIANS);
            this.pitch = angles.getPitch(AngleUnit.RADIANS);
            this.roll = angles.getRoll(AngleUnit.RADIANS);
        }
        {
            this.xRotationRate = angularVelocity.xRotationRate;
            this.yRotationRate = angularVelocity.yRotationRate;
            this.zRotationRate = angularVelocity.zRotationRate;
        }
    }
}
