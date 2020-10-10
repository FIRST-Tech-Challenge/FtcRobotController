package com.technototes.subsystem;

import com.qualcomm.robotcore.util.Range;

public interface HolomnicDrivebaseSubsystem extends DrivebaseSubsystem {

    default void joystickDrive(double x, double y, double rotation){
        joystickDriveWithGyro(x, y, rotation, 0);
    }

    default void joystickDriveWithGyro(double x, double y, double rotation, double gyroAngle) {
        double speed = Range.clip(Math.abs(Math.sqrt(x*x+y*y)), 0, 1);
        double headingRad = Math.toRadians(gyroAngle);
        double angle = -Math.atan2(y, x) + headingRad - Math.PI/4;
        drive(speed, angle, rotation);
    }

    default void drive(double speed, double angle, double rotation) {
        double x = Math.cos(angle) * speed;
        double y = Math.sin(angle) * speed;

        double powerCompY = -(x + y);
        double powerCompX = x - y;

        speed = Range.clip(speed + Math.abs(rotation), 0, 1);

        double flPower = powerCompY - powerCompX - 2*rotation;
        double frPower = -powerCompY - powerCompX - 2*rotation;
        double rlPower = powerCompY + powerCompX - 2*rotation;
        double rrPower = -powerCompY + powerCompX - 2*rotation;

        double scale = getScale(flPower, frPower, rlPower, rrPower);
        scale = scale == 0 ? 0 : speed/scale;
        scale = Math.cbrt(scale);
        drive(flPower*scale, frPower*scale,rlPower*scale, rrPower*scale);
    }

    void drive(double flSpeed, double frSpeed, double rlSpeed, double rrSpeed);

    @Override
    default void stop() {
        drive(0, 0, 0, 0);
    }
}
