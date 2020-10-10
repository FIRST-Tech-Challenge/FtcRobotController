package com.technototes.library.subsystem.drivebase;

import com.technototes.library.hardware.motor.EncodedMotor;
import com.technototes.library.hardware.motor.Motor;
import com.technototes.subsystem.HolomnicDrivebaseSubsystem;

import java.util.function.DoubleSupplier;
//dont use incomplete
@Deprecated
public class SwerveDrivebaseSubsystem<T extends Motor, U extends EncodedMotor> extends DrivebaseSubsystem<T> implements HolomnicDrivebaseSubsystem {
    public U steer1, steer2, steer3, steer4;
    //int numMod
    public SwerveDrivebaseSubsystem(T[] d, U[] s) {
        super(d);
        steer1 = s[0];
        steer2 = s[1];
        if(s.length == 4){
            steer3 = s[2];
            steer4 = s[3];
        }
    }

    public SwerveDrivebaseSubsystem(DoubleSupplier gyro, T... d) {
        super(gyro, d);
    }

    @Override
    public void drive(double flSpeed, double frSpeed, double rlSpeed, double rrSpeed) {

    }
}
