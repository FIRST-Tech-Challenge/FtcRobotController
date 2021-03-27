package com.technototes.subsystem;

import com.qualcomm.robotcore.util.Range;

/** An interface to extend holonomic drivebases from
 * @author Alex Stedman
 */
public interface HolonomicDrivebaseSubsystem extends DrivebaseSubsystem {

    /** Basic joystick drive function
     *
     * @param x Left/Right Motion (pass joystick left stick x usually)
     * @param y Up/Down Motion (pass joystick left stick y usually)
     * @param rotation Rotational Motion (pass joystick right stick x usually)
     */
    default void joystickDrive(double x, double y, double rotation){
        joystickDriveWithGyro(x, y, rotation, 0);
    }
    /** Basic joystick drive function with gyro
     *
     * @param x Left/Right Motion (pass joystick left stick x usually)
     * @param y Up/Down Motion (pass joystick left stick y usually)
     * @param rotation Rotational Motion (pass joystick right stick x usually)
     * @param gyroAngle The gyro value (degrees) (0 is initial robot orientation)
     */
    default void joystickDriveWithGyro(double x, double y, double rotation, double gyroAngle) {
        double speed = Range.clip(Math.abs(Math.hypot(x, y)), 0, 1);
        double headingRad = Math.toRadians(gyroAngle);
        double angle = -Math.atan2(y, x) + headingRad - Math.PI/4;
        drive(speed, angle, rotation);
    }

    /** Basic drive function
     *
     * @param speed Speed to drive at
     * @param angle Angle to drive at
     * @param rotation Amount to rotate/turn
     */
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

    /** Function to set motor powers for driving (do scale calculations here)
     *
     * @param flSpeed Speed to set front left motor to
     * @param frSpeed Speed to set front right motor to
     * @param rlSpeed Speed to set rear left motor to
     * @param rrSpeed Speed to set rear right motor to
     */
    void drive(double flSpeed, double frSpeed, double rlSpeed, double rrSpeed);

    /** Stop Drivebase
     *
     */
    @Override
    default void stop() {
        drive(0, 0, 0, 0);
    }
}
