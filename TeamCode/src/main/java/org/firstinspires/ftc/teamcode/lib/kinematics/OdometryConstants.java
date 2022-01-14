package org.firstinspires.ftc.teamcode.lib.kinematics;

import edu.wpi.first.wpilibj.geometry.Translation2d;

public class OdometryConstants {
    public final Translation2d leftWheel;
    public final Translation2d rightWheel;
    public final Translation2d horizontalWheel;
    public final double wheel_diameter;
    public final double ticks_per_revolution;

    public final double meters_per_tick;

    public OdometryConstants(Translation2d leftWheel, Translation2d rightWheel, Translation2d horizontalWheel, double wheel_diameter, double ticks_per_revolution) {
        this.leftWheel = leftWheel;
        this.rightWheel = rightWheel;
        this.horizontalWheel = horizontalWheel;
        this.wheel_diameter = wheel_diameter;
        this.ticks_per_revolution = ticks_per_revolution;

        meters_per_tick = wheel_diameter * Math.PI / ticks_per_revolution;
    }

    public double getVerticalWheelsDistance() {
        return leftWheel.getDistance(rightWheel);
    }

    public double getHorizontalWheelOffset() {
        return (leftWheel.plus(rightWheel).div(2)).getDistance(horizontalWheel);
    }
}
