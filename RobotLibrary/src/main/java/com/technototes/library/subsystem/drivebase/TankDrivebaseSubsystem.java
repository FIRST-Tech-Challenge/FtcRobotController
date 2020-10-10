package com.technototes.library.subsystem.drivebase;

import com.technototes.control.gamepad.GamepadStick;
import com.technototes.library.control.gamepad.old.OldCommandGamepad;
import com.technototes.library.hardware.motor.Motor;
import com.technototes.subsystem.NonHolomnicDrivebaseSubsystem;

public class TankDrivebaseSubsystem<T extends Motor> extends DrivebaseSubsystem<T> implements NonHolomnicDrivebaseSubsystem {
    private T leftSide, rightSide;

    public TankDrivebaseSubsystem(T l, T r) {
        super(l, r);
        leftSide = l;
        rightSide = r;
    }

    @Override
    public void drive(double l, double r) {
        leftSide.setSpeed(l*getSpeed());
        rightSide.setSpeed(r*getSpeed());
    }
}
