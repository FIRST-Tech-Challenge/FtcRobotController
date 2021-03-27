package com.technototes.library.subsystem.drivebase;

import com.technototes.library.hardware.motor.Motor;
import com.technototes.subsystem.HolonomicDrivebaseSubsystem;

import java.util.function.DoubleSupplier;

/** Class for mecanum/xdrive drivebases
 * @author Alex Stedman
 * @param <T> The motor type for the subsystem
 */
public class MecanumDrivebaseSubsystem<T extends Motor<?>> extends DrivebaseSubsystem<T> implements HolonomicDrivebaseSubsystem {
    /** Drive motors
     *
     */
    public T flMotor, frMotor, rlMotor, rrMotor;

    /** Create mecanum drivebase
     *
     * @param flMotor The front left motor for the drivebase
     * @param frMotor The front right motor for the drivebase
     * @param rlMotor The rear left motor for the drivebase
     * @param rrMotor The rear right motor for the drivebase
     */
    public MecanumDrivebaseSubsystem(T flMotor, T frMotor, T rlMotor, T rrMotor) {
        super(flMotor, frMotor, rlMotor, rrMotor);
        this.flMotor = flMotor;
        this.frMotor = frMotor;
        this.rlMotor = rlMotor;
        this.rrMotor = rrMotor;
    }

    /** Create mecanum drivebase
     *
     * @param gyro The gyro supplier
     * @param flMotor The front left motor for the drivebase
     * @param frMotor The front right motor for the drivebase
     * @param rlMotor The rear left motor for the drivebase
     * @param rrMotor The rear right motor for the drivebase
     */
    public MecanumDrivebaseSubsystem(DoubleSupplier gyro, T flMotor, T frMotor, T rlMotor, T rrMotor) {
        super(gyro, flMotor, frMotor, rlMotor, rrMotor);
        this.flMotor = flMotor;
        this.frMotor = frMotor;
        this.rlMotor = rlMotor;
        this.rrMotor = rrMotor;
    }
    
    @Override
    public void drive(double flSpeed, double frSpeed, double rlSpeed, double rrSpeed) {
        flMotor.setSpeed(flSpeed*getSpeed());
        frMotor.setSpeed(frSpeed*getSpeed());
        rlMotor.setSpeed(rlSpeed*getSpeed());
        rrMotor.setSpeed(rrSpeed*getSpeed());
    }
}
