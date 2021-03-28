package com.technototes.library.subsystem.drivebase;

import com.technototes.library.hardware.motor.Motor;
import com.technototes.library.subsystem.Subsystem;

import java.util.function.DoubleSupplier;

/** Class for DriveBase subsystems
 * @author Alex Stedman The motors for the drivebase
 * @param <T> The type of motors for the drivebase
 */
public abstract class DrivebaseSubsystem<T extends Motor<?>> extends Subsystem<T> implements com.technototes.subsystem.DrivebaseSubsystem {
    protected DoubleSupplier gyroSupplier = () -> 0;

    /** The default drive speeds
     *
     */
    @Deprecated
    public SampleDriveSpeed driveSpeed = SampleDriveSpeed.NORMAL;

    /** Create a drivebase subsystem
     *
     * @param motors The drive motors
     */
    public DrivebaseSubsystem(T... motors) {
        super(motors);
    }
    /** Create a drivebase subsystem
     * @param gyro The gyro supplier
     * @param motors The drive motors
     */
    public DrivebaseSubsystem(DoubleSupplier gyro, T... motors) {
        super(motors);
        gyroSupplier = gyro;
    }

    /** Get the Gyro angle
     *
     * @return Gyro angle from supplier
     */
    public double getGyro(){
        return gyroSupplier.getAsDouble();
    }

}
