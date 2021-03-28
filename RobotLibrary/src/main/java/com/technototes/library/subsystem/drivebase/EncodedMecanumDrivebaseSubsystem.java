package com.technototes.library.subsystem.drivebase;

import com.technototes.library.hardware.motor.EncodedMotor;

import java.util.function.DoubleSupplier;

/** mecanum but uses encoded motors for driving and uses velocity over voltage
 * totally not to make tristan happy
 * @author Alex Stedman
 */
public class EncodedMecanumDrivebaseSubsystem extends MecanumDrivebaseSubsystem<EncodedMotor<?>> {
    public EncodedMecanumDrivebaseSubsystem(EncodedMotor<?> flMotor, EncodedMotor<?> frMotor, EncodedMotor<?> rlMotor, EncodedMotor<?> rrMotor) {
        super(flMotor, frMotor, rlMotor, rrMotor);
    }

    public EncodedMecanumDrivebaseSubsystem(DoubleSupplier gyro, EncodedMotor<?> flMotor, EncodedMotor<?> frMotor, EncodedMotor<?> rlMotor, EncodedMotor<?> rrMotor) {
        super(gyro, flMotor, frMotor, rlMotor, rrMotor);
    }
    @Override
    public void drive(double flSpeed, double frSpeed, double rlSpeed, double rrSpeed) {
        flMotor.setVelocity(flSpeed*getSpeed());
        frMotor.setVelocity(frSpeed*getSpeed());
        rlMotor.setVelocity(rlSpeed*getSpeed());
        rrMotor.setVelocity(rrSpeed*getSpeed());
    }


}
