package com.technototes.library.subsystem.drivebase;

import com.technototes.library.hardware.motor.Motor;
import com.technototes.library.subsystem.Subsystem;
import com.technototes.subsystem.DrivebaseSubsystem.DriveSpeed;

import java.util.function.DoubleSupplier;

public abstract class DrivebaseSubsystem<T extends Motor> extends Subsystem<T> implements com.technototes.subsystem.DrivebaseSubsystem {
    public DoubleSupplier gyroSupplier = () -> 0;
    public DriveSpeed driveSpeed = DriveSpeed.NORMAL;


    public DrivebaseSubsystem(T... d) {
        super(d);
    }
    public DrivebaseSubsystem(DoubleSupplier s, T... d) {
        super(d);
        gyroSupplier = s;
    }

    public double getGyro(){
        return gyroSupplier.getAsDouble();
    }

    @Override
    public void setDriveSpeed(DriveSpeed ds) {
        driveSpeed = ds;
    }

    @Override
    public DriveSpeed getDriveSpeed() {
        return driveSpeed;
    }

}
