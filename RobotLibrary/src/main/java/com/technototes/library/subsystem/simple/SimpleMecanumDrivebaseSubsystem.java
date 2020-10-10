package com.technototes.library.subsystem.simple;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.technototes.library.hardware.motor.Motor;
import com.technototes.library.subsystem.drivebase.MecanumDrivebaseSubsystem;

import java.util.function.DoubleSupplier;
@Deprecated
public class SimpleMecanumDrivebaseSubsystem extends MecanumDrivebaseSubsystem<Motor> {
    public SimpleMecanumDrivebaseSubsystem(Motor<DcMotor> flMotor, Motor<DcMotor> frMotor, Motor<DcMotor> rlMotor, Motor<DcMotor> rrMotor) {
        super(flMotor, frMotor, rlMotor, rrMotor);
    }

    public SimpleMecanumDrivebaseSubsystem(DoubleSupplier d, Motor<DcMotor> flMotor, Motor<DcMotor> frMotor, Motor<DcMotor> rlMotor, Motor<DcMotor> rrMotor) {
        super(d, flMotor, frMotor, rlMotor, rrMotor);
    }
}
