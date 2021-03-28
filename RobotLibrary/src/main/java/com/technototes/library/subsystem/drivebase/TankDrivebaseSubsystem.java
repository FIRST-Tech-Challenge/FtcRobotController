package com.technototes.library.subsystem.drivebase;

import com.technototes.library.hardware.motor.Motor;
import com.technototes.subsystem.NonHolomonicDrivebaseSubsystem;

/** Class for drivebase subsystems
 * @author Alex Stedman
 * @param <T> The type of motor for the drivebase
 */
public class TankDrivebaseSubsystem<T extends Motor<?>> extends DrivebaseSubsystem<T> implements NonHolomonicDrivebaseSubsystem {
    /** Drive motors
     *
     */
    public T leftSide, rightSide;

    /** Create tank drivebase
     *
     * @param leftMotor The motor/motorgroup for the left side of the drivebase
     * @param rightMotor The motor/motorgroup for the right side of the drivebase
     */
    public TankDrivebaseSubsystem(T leftMotor, T rightMotor) {
        super(leftMotor, rightMotor);
        leftSide = leftMotor;
        rightSide = rightMotor;
    }


    @Override
    public void drive(double l, double r) {
        leftSide.setSpeed(l*getSpeed());
        rightSide.setSpeed(r*getSpeed());
    }
}
